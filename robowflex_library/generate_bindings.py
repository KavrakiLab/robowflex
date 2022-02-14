import argparse
import re
from collections import defaultdict
from dataclasses import dataclass
from itertools import repeat
from pathlib import Path
from typing import Dict, Iterable, List, Set, Tuple

import clang.cindex
from clang.cindex import (AccessSpecifier, AvailabilityKind, Cursor, CursorKind,
                          Index, TranslationUnit, Type, TypeKind)


def load_data(compilation_database_file: Path,
              header_paths: List[Path]) -> Tuple[Index, List[TranslationUnit]]:
    # Parse the given input file
    index = clang.cindex.Index.create()
    comp_db = clang.cindex.CompilationDatabase.fromDirectory(
        compilation_database_file)
    translation_units = []
    for header_path in header_paths:
        commands = comp_db.getCompileCommands(header_path)
        file_args = []
        for command in commands:
            for argument in command.arguments:
                file_args.append(argument)

        file_args = file_args[2:-1]
        # NOTE: Not sure why this needs to be manually included, but we don't find stddef otherwise
        file_args.append('-I/usr/lib/clang/13.0.1/include')
        translation_units.append(
            index.parse(header_path,
                        file_args,
                        options = TranslationUnit.PARSE_SKIP_FUNCTION_BODIES))
    return index, translation_units


def get_nodes_from_file(nodes: Iterable[Cursor],
                        file_name: str) -> List[Cursor]:
    return list(filter(lambda n: n.location.file.name == file_name, nodes))


def get_nodes_with_kind(nodes: Iterable[Cursor],
                        node_kinds: Iterable[CursorKind]) -> List[Cursor]:
    return list(filter(lambda n: n.kind in node_kinds, nodes))


# This approach to snake-case conversion adapted from:
# https://stackoverflow.com/questions/1175208/elegant-python-function-to-convert-camelcase-to-snake-case
SPECIAL_KEYWORDS = re.compile(r'ID|YAML|SRDF|URDF|JSON|IK')
NAIVE_CAMEL_CASE = re.compile(r'(.)([A-Z][a-z]+)')
CAMEL_CASE_UNDERSCORES = re.compile(r'__([A-Z])')
ADVANCED_CAMEL_CASE = re.compile(r'([a-z0-9])([A-Z])')


def downcase_keywords(kwd_match: re.Match) -> str:
    kwd = kwd_match.group(0)
    return kwd[0] + kwd[1:].lower()


def to_snake_case(camel_name: str) -> str:
    snake_name = SPECIAL_KEYWORDS.sub(downcase_keywords, camel_name)
    snake_name = NAIVE_CAMEL_CASE.sub(r'\1_\2', snake_name)
    snake_name = CAMEL_CASE_UNDERSCORES.sub(r'_\1', snake_name)
    snake_name = ADVANCED_CAMEL_CASE.sub(r'\1_\2', snake_name)
    return snake_name.lower()


def generate_function_pointer_signature(qualified_name: str,
                                        function_node: Cursor,
                                        is_static: bool = False,
                                        class_node: Cursor = None) -> str:
    signature = f'{function_node.type.get_result().spelling} ({qualified_name + "::*" if class_node and not is_static else "*"})('
    for typ in function_node.type.argument_types():
        signature += f'{typ.get_canonical().spelling}, '

    signature = signature[:-2] + ')'
    if function_node.is_const_method():
        signature += ' const'

    return signature


def generate_overloads(name: str,
                       nodes: Iterable[Cursor],
                       qualified_name: str,
                       class_node: Cursor = None) -> List[str]:
    overloads = []
    for function_node in nodes:
        is_static = function_node.is_static_method()
        function_pointer_signature = generate_function_pointer_signature(
            qualified_name, function_node, is_static, class_node)
        if is_static:
            # NOTE: This is a gross hack but seems necessary given limitations of pybind11
            overload_name = f'{name}_static'
        else:
            overload_name = name

        overloads.append(
            f'{".def_static" if is_static else ".def"}("{overload_name}", static_cast<{function_pointer_signature}>(&{qualified_name}::{function_node.spelling}))'
        )

    return overloads


def get_base_type_name(typ: Type) -> str:
    type_name_chunks = typ.spelling.split(' ')
    # Check for const and reference
    return ' '.join(chunk if chunk not in ['const', '&'] else ''
                    for chunk in type_name_chunks).strip()


def generate_operator_methods(operator: str,
                              nodes: Iterable[Cursor]) -> List[str]:
    methods = []
    for method_node in nodes:
        argument_type_names = [
            get_base_type_name(typ)
            for typ in method_node.type.argument_types()
        ]
        if len(argument_type_names) >= 2:
            print(method_node.spelling, argument_type_names)

        assert (len(argument_type_names) < 2)
        if len(argument_type_names) == 0:
            methods.append(f'.def({operator}py::self)')
        else:
            methods.append(
                f'.def(py::self {operator} {argument_type_names[0]}())')

    return methods


def is_exposed(node: Cursor) -> bool:
    return node.access_specifier == AccessSpecifier.PUBLIC    # type: ignore


# TODO: This could be more efficient if we called get_children once per class and iterated over it
# multiple times, at the cost of threading that list through in the parameters
# TODO: Could also improve efficiency with stronger preference for iterators over explicit lists


def get_exposed_fields(class_node: Cursor) -> List[Cursor]:
    fields = get_nodes_with_kind(class_node.get_children(),
                                 [CursorKind.FIELD_DECL])    # type: ignore
    return list(filter(is_exposed, fields))


def get_exposed_static_variables(class_node: Cursor) -> List[Cursor]:
    fields = get_nodes_with_kind(class_node.get_children(),
                                 [CursorKind.VAR_DECL])    # type: ignore
    return list(filter(is_exposed, fields))


def get_exposed_methods(class_node: Cursor) -> List[Cursor]:
    methods = get_nodes_with_kind(class_node.get_children(),
                                  [CursorKind.CXX_METHOD])    # type: ignore
    return list(filter(is_exposed, methods))


# TODO: Handle default arguments
# TODO: Maybe generate keyword args?
# TODO: Maybe export constants?


@dataclass
class Binding:
    name: str
    is_class: bool
    dependencies: List[str]
    body: List[str]

    def __repr__(self) -> str:
        return f'Binding(name={self.name}, is_class={self.is_class}, dependencies={self.dependencies})'


def generate_constructor_wrapper(qualified_name: str,
                                 argument_types: List[Type]) -> str:
    '''Generate an anonymous wrapper for constructors taking double-pointer arguments.'''
    # TODO: Could generalize this to wrap functions with double-pointer arguments, but I haven't found
    # TODO: This whole function should be rewritten; it's messy and inefficient
    # any in the codebase yet
    modified_arg_types = []
    modified_arg_indices = set()
    for i, typ in enumerate(argument_types):
        canonical_typ = typ.get_canonical()
        pointee_type = canonical_typ.get_pointee().get_pointee()
        if pointee_type.kind != TypeKind.INVALID:    # type: ignore
            # See pybind11 known limitation #1
            if pointee_type.kind in (
                    TypeKind.CHAR_U,    # type: ignore
                    TypeKind.UCHAR):    # type: ignore
                vec_elem_type = 'std::string'
            else:
                vec_elem_type = canonical_typ.get_pointee().spelling

            modified_arg_types.append(f'std::vector<{vec_elem_type}>')
            modified_arg_indices.add(i)
        else:
            modified_arg_types.append(canonical_typ.spelling)

    arg_names = [f'arg_{i}' for i in range(len(modified_arg_types))]
    lambda_args = [
        typ + ' ' + arg_name
        for typ, arg_name in zip(modified_arg_types, arg_names)
    ]

    invocation_expr = ', '.join(
        arg_name if i not in
        modified_arg_indices else f'convertVec({arg_name}).data()'
        for i, arg_name in enumerate(arg_names))

    return f'.def(py::init([]({", ".join(lambda_args)}) {{return {qualified_name}({invocation_expr});}}))'


def generate_constructors(class_node: Cursor, qualified_name: str) -> List[str]:
    constructors = []
    for constructor_node in get_nodes_with_kind(
            class_node.get_children(),
        [CursorKind.CONSTRUCTOR]):    # type: ignore
        if constructor_node.availability != AvailabilityKind.NOT_AVAILABLE:    # type: ignore
            # TODO: This could be cleaner
            if any(typ.get_pointee().get_pointee().kind !=
                   TypeKind.INVALID    # type: ignore
                   for typ in constructor_node.type.argument_types()):
                constructors.append(
                    generate_constructor_wrapper(
                        qualified_name, constructor_node.type.argument_types()))
            else:
                constructors.append(
                    f".def(py::init<{', '.join([typ.get_canonical().spelling for typ in constructor_node.type.argument_types()])}>())"
                )

    return constructors


def generate_methods(class_node: Cursor, qualified_name: str) -> List[str]:
    class_method_nodes = defaultdict(list)
    for method_node in get_exposed_methods(class_node):
        class_method_nodes[to_snake_case(
            method_node.spelling)].append(method_node)

    methods = []
    for method_name, method_nodes in class_method_nodes.items():
        # Handle operators
        if method_name[:8] == 'operator':
            operator = method_name[8:].strip()
            if operator[:
                        3] == 'new' or operator[:
                                                6] == 'delete' or operator == '=':
                continue

            methods.extend(generate_operator_methods(operator, method_nodes))
        elif len(method_nodes) > 1:
            methods.extend(
                generate_overloads(method_name, method_nodes, qualified_name,
                                   class_node))
        else:
            methods.append(
                f'{".def_static" if method_nodes[0].is_static_method() else ".def"}("{method_name}", &{qualified_name}::{method_nodes[0].spelling})'
            )

    return methods


def generate_fields(class_node: Cursor, qualified_name: str) -> List[str]:
    fields = []
    # Instance fields
    for field_node in get_exposed_fields(class_node):
        field_binder = '.def_readwrite'
        if field_node.type.is_const_qualified():
            field_binder = '.def_readonly'

        fields.append(
            f'{field_binder}("{to_snake_case(field_node.spelling)}", &{qualified_name}::{field_node.spelling})'
        )

    # Static variables
    for static_var_node in get_exposed_static_variables(class_node):
        var_binder = '.def_readwrite'
        if static_var_node.type.is_const_qualified():
            var_binder = '.def_readonly_static'

        fields.append(
            f'{var_binder}("{to_snake_case(static_var_node.spelling)}", &{qualified_name}::{static_var_node.spelling})'
        )

    return fields


def get_nested_types(class_node: Cursor) -> List[Cursor]:
    return get_nodes_with_kind(
        class_node.get_children(),
        [CursorKind.CLASS_DECL, CursorKind.STRUCT_DECL])    # type: ignore


def generate_class(class_node: Cursor,
                   ns_name: str,
                   pointer_names: Set[str],
                   parent_class: str = None) -> List[Binding]:
    # Handle forward declarations
    class_definition_node = class_node.get_definition()
    if class_definition_node is None or class_definition_node != class_node:
        return []

    parent_object = f'py_{parent_class}' if parent_class else 'm'

    superclasses = []
    for superclass_node in get_nodes_with_kind(
            class_node.get_children(),
        [CursorKind.CXX_BASE_SPECIFIER]):    # type: ignore
        superclasses.append(superclass_node.type.spelling)

    superclass_string = ''
    qualified_name = f'{ns_name}::{parent_class + "::" if parent_class else ""}{class_node.spelling}'
    # NOTE: We assume that everything uses shared_ptr as its holder type for simplicity
    pointer_string = f', std::shared_ptr<{qualified_name}>'
    class_output = [f'// Bindings for class {qualified_name}']
    filtered_superclasses = [
        superclass for superclass in superclasses if superclass[:5] != 'std::'
    ]
    if filtered_superclasses:
        superclass_string = f', {",".join(filtered_superclasses)}'

    nested_output: List[Binding] = []
    if 'std::exception' in superclasses:
        class_output.append(
            f'py::register_exception<{qualified_name}>({parent_object}, "{class_node.spelling}"{superclass_string})'
        )
    else:
        class_output.append(
            f'py::class_<{qualified_name}{superclass_string}{pointer_string}>({parent_object}, "{class_node.spelling}")'
        )
        # Constructors
        if not class_node.is_abstract_record():
            class_output.extend(
                generate_constructors(class_node, qualified_name))

        # Methods
        class_output.extend(generate_methods(class_node, qualified_name))

        # Fields
        class_output.extend(generate_fields(class_node, qualified_name))

        # Nested types
        for nested_type_node in get_nested_types(class_node):
            nested_output.extend(
            # NOTE: If we ever have doubly-nested classes, this could be wrong - would need to pass
            # qualified_name instead
                generate_class(nested_type_node, ns_name, pointer_names,
                               class_node.spelling))

        if nested_output:
            class_output[
                1] = f'py::class_<{qualified_name}{superclass_string}{pointer_string}> py_{class_node.spelling}({parent_object}, "{class_node.spelling}");'
            class_output[2] = f'py_{class_node.spelling}{class_output[2]}'

    class_output.append(';')
    class_binding = Binding(qualified_name,
                            True,
                            dependencies = [],
                            body = class_output)
    class_binding.dependencies.extend(filtered_superclasses)
    if parent_class:
        class_binding.dependencies.append(f'{ns_name}::{parent_class}')

    return [class_binding] + nested_output


def get_namespaces(top_level_node: Cursor) -> List[Cursor]:
    if top_level_node.kind == CursorKind.NAMESPACE:    # type: ignore
        return [top_level_node]
    else:
        return get_nodes_with_kind(top_level_node.get_children(),
                                   [CursorKind.NAMESPACE])    # type: ignore


def get_pointer_defs(top_level_node: Cursor) -> Set[str]:
    typedef_nodes = get_nodes_with_kind(
        top_level_node.get_children(),
        [CursorKind.TYPEDEF_DECL])    # type: ignore
    return {
        node.spelling
        for node in typedef_nodes if node.spelling[-3:].lower() == 'ptr'
    }


def bind_classes(top_level_node: Cursor) -> List[Binding]:
    output = []
    for ns in get_namespaces(top_level_node):
        pointer_names = get_pointer_defs(ns)
        for class_node in get_nodes_with_kind(
                ns.get_children(),
            [CursorKind.CLASS_DECL, CursorKind.STRUCT_DECL]):    # type: ignore
            output.extend(generate_class(class_node, ns.spelling,
                                         pointer_names))
    return output


# TODO Templates
def bind_functions(top_level_node: Cursor) -> List[Binding]:
    function_bindings = []
    for ns in get_namespaces(top_level_node):
        ns_functions: Dict[str, List[Cursor]] = defaultdict(list)
        # We do this in two stages to handle overloads
        for function_node in get_nodes_with_kind(
                ns.get_children(),
            [CursorKind.FUNCTION_DECL]):    # type: ignore
            ns_functions[to_snake_case(
                function_node.spelling)].append(function_node)

        for function_name, function_nodes in ns_functions.items():
            if len(function_nodes) > 1:
                function_body = ['m']
                function_body.extend(
                    generate_overloads(function_name, function_nodes,
                                       ns.spelling))
                function_body.append(';')
                function_bindings.append(
                    Binding(name = function_name,
                            is_class = False,
                            dependencies = [],
                            body = function_body))
            else:
                function_bindings.append(
                    Binding(
                        name = function_name,
                        is_class = False,
                        dependencies = [],
                        body = [
                            f'm.def("{function_name}", &{ns.spelling}::{function_nodes[0].spelling});'
                        ]))
    return function_bindings


def print_tree(root: Cursor, depth: int = 0):
    print(f'{"".join(["  "] * depth)}{root.displayname}: {root.kind}')
    for child in root.get_children():
        print_tree(child, depth + 1)


def generate_bindings(translation_unit: TranslationUnit) -> List[Binding]:
    bindings = []
    for diagnostic in translation_unit.diagnostics:
        print(diagnostic.format())

    file_nodes = get_nodes_from_file(translation_unit.cursor.get_children(),
                                     translation_unit.spelling)
    for node in file_nodes:
        bindings.extend(bind_classes(node))
        bindings.extend(bind_functions(node))

    return bindings


def toposort_bindings(bindings: List[Binding]) -> List[str]:
    output: List[str] = []
    class_bindings = [
        binding for binding in bindings
        if binding.is_class and binding.dependencies
    ]
    frontier = [
        binding for binding in bindings
        if binding.is_class and not binding.dependencies
    ]

    while frontier:
        next_binding = frontier.pop()
        output.extend(next_binding.body)
        new_class_bindings = []
        for binding in class_bindings:
            if next_binding.name in binding.dependencies:
                binding.dependencies.remove(next_binding.name)

            if not binding.dependencies:
                frontier.append(binding)
            else:
                new_class_bindings.append(binding)

        class_bindings = new_class_bindings

    for binding in filter(lambda b: not b.is_class, bindings):
        output.extend(binding.body)

    for foo in output:
        assert not isinstance(foo, Binding), str(foo)
    return output


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('-m',
                            '--module-name',
                            help = 'The name of the output Python module',
                            type = Path)
    arg_parser.add_argument('-o',
                            '--output-file',
                            help = 'Output file path',
                            type = Path)
    arg_parser.add_argument(
        '-c',
        '--compilation-database',
        help = 'Directory containing the project compile_commands.json',
        type = Path)
    arg_parser.add_argument('headers',
                            metavar = 'HEADER',
                            help = 'Header files to generate bindings for',
                            nargs = '+',
                            type = Path)
    args = arg_parser.parse_args()
    prefix = [
        r'#include <pybind11/pybind11.h>', r'#include <pybind11/operators.h>',
        r'''
template <typename ElemT, typename ResultT=ElemT>
std::vector<ResultT> convertVec(const std::vector<ElemT>& vec) {
  return vec;
}
template<>
std::vector<const char*> convertVec(const std::vector<std::string>& vec) {
  std::vector<const char*> result;
  result.reserve(vec.size());
  for(const auto& str: vec) {
    result.push_back(str.c_str());
  }
  return result;
}'''
    ]

    body = []

    print('Parsing header files...')
    index, translation_units = load_data(args.compilation_database,
                                         args.headers)

    print('Generating bindings...')
    bindings = []
    for tu in translation_units:
        bindings.extend(generate_bindings(tu))

    # Put in dependency order:
    bodies = toposort_bindings(bindings)

    prefix.extend(f"#include <{header_path.relative_to('include')}>"
                  for header_path in args.headers)
    prefix.extend([
        'namespace py = pybind11;', f'PYBIND11_MODULE({args.module_name}, m) {{'
    ])
    output = prefix + bodies + ['}']
    print(f'Outputting bindings to {args.output_file}')
    with open(args.output_file, 'w') as output_file:
        output_file.writelines([
            line for pair in zip(output, repeat('\n', len(output)))
            for line in pair
        ])
