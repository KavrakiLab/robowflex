import argparse
import re
from collections import defaultdict
from itertools import repeat
from pathlib import Path
from typing import Dict, Iterable, List, Set, Tuple

import clang.cindex
from clang.cindex import (AccessSpecifier, Cursor, CursorKind, Index,
                          TranslationUnit, Type)


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


def generate_function_pointer_signature(ns_name: str,
                                        function_node: Cursor,
                                        class_node: Cursor = None) -> str:
    signature = f'{function_node.type.get_result().spelling} ({ns_name + "::" + class_node.spelling + "::*" if class_node else "*"})('
    for typ in function_node.type.argument_types():
        signature += f'{typ.spelling}, '

    signature = signature[:-2] + ')'
    return signature


def generate_overloads(name: str,
                       nodes: Iterable[Cursor],
                       ns_name: str,
                       class_node: Cursor = None) -> List[str]:
    overloads = []
    for function_node in nodes:
        function_pointer_signature = generate_function_pointer_signature(
            ns_name, function_node, class_node)
        overloads.append(
            f'.def("{name}", static_cast<{function_pointer_signature}>(&{ns_name}::{class_node.spelling + "::" if class_node else ""}{function_node.spelling}))'
        )

    return overloads


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


def generate_constructors(class_node: Cursor) -> List[str]:
    constructors = []
    for constructor_node in get_nodes_with_kind(
            class_node.get_children(),
        [CursorKind.CONSTRUCTOR]):    # type: ignore
        constructors.append(
            f".def(py::init<{', '.join([typ.spelling for typ in constructor_node.type.argument_types()])}>())"
        )

    return constructors


def generate_methods(class_node: Cursor, ns_name: str) -> List[str]:
    class_method_nodes = defaultdict(list)
    for method_node in get_exposed_methods(class_node):
        class_method_nodes[to_snake_case(
            method_node.spelling)].append(method_node)

    methods = []
    for method_name, method_nodes in class_method_nodes.items():
        if len(method_nodes) > 1:
            methods.extend(
                generate_overloads(method_name, method_nodes, ns_name,
                                   class_node))
        else:
            methods.append(
                f'.def("{method_name}", &{ns_name}::{class_node.spelling}::{method_nodes[0].spelling})'
            )

    return methods


def generate_fields(class_node: Cursor, ns_name: str) -> List[str]:
    fields = []
    # Instance fields
    for field_node in get_exposed_fields(class_node):
        field_binder = '.def_readwrite'
        if field_node.type.is_const_qualified():
            field_binder = '.def_readonly'

        fields.append(
            f'{field_binder}("{to_snake_case(field_node.spelling)}", &{ns_name}::{class_node.spelling}::{field_node.spelling})'
        )

    # Static variables
    for static_var_node in get_exposed_static_variables(class_node):
        var_binder = '.def_readwrite'
        if static_var_node.type.is_const_qualified():
            var_binder = '.def_readonly_static'

        fields.append(
            f'{var_binder}("{to_snake_case(static_var_node.spelling)}", &{ns_name}::{class_node.spelling}::{static_var_node.spelling})'
        )

    return fields


def get_nested_types(class_node: Cursor) -> List[Cursor]:
    return get_nodes_with_kind(
        class_node.get_children(),
        [CursorKind.CLASS_DECL, CursorKind.STRUCT_DECL])    # type: ignore


def generate_class(class_node: Cursor,
                   ns_name: str,
                   pointer_names: Set[str],
                   parent_class: str = None) -> List[str]:
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
    if superclasses:
        superclass_string = f', {",".join(superclasses)}'

    pointer_string = ''
    pointer_type_name = f'{class_node.spelling}Ptr'
    if pointer_type_name in pointer_names:
        pointer_string = f', {ns_name}::{pointer_type_name}'

    class_output = [
        f'// Bindings for class {ns_name}',
        f'py::class_<{ns_name}::{class_node.spelling}{superclass_string}{pointer_string}>({parent_object}, "{class_node.spelling}")'
    ]
    # Constructors
    class_output.extend(generate_constructors(class_node))

    # Methods
    class_output.extend(generate_methods(class_node, ns_name))

    # Fields
    class_output.extend(generate_fields(class_node, ns_name))

    # Nested types
    nested_output = []
    for nested_type_node in get_nested_types(class_node):
        nested_output.extend(
            generate_class(nested_type_node, ns_name, class_node.spelling))

    if nested_output:
        class_output[
            1] = f'py::class_<{ns_name}::{class_node.spelling}> py_{class_node.spelling}(m, "{class_node.spelling}")'

    class_output.append(';')
    return class_output


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


def bind_classes(top_level_node: Cursor) -> List[str]:
    output = []
    for ns in get_namespaces(top_level_node):
        pointer_names = get_pointer_defs(ns)
        for class_node in get_nodes_with_kind(
                ns.get_children(),
            [CursorKind.CLASS_DECL, CursorKind.STRUCT_DECL]):    # type: ignore
            output.extend(generate_class(class_node, ns.spelling,
                                         pointer_names))
    return output


def bind_functions(top_level_node: Cursor) -> List[str]:
    output = []
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
                output.append('m')
                output.extend(
                    generate_overloads(function_name, function_nodes,
                                       ns.spelling))
                output.append(';')
            else:
                output.append(
                    f'm.def("{function_name}", &{ns.spelling}::{function_nodes[0].spelling});'
                )
    return output


def print_tree(root: Cursor, depth: int = 0):
    print(f'{"".join(["  "] * depth)}{root.displayname}: {root.kind}')
    for child in root.get_children():
        print_tree(child, depth + 1)


def generate_bindings(header_path: Path,
                      translation_unit: TranslationUnit) -> List[str]:
    bindings = []
    for diagnostic in translation_unit.diagnostics:
        print(diagnostic.format())

    file_nodes = get_nodes_from_file(translation_unit.cursor.get_children(),
                                     translation_unit.spelling)
    bindings.append(f'// Bindings for {header_path}')
    for node in file_nodes:
        bindings.extend(bind_classes(node))
        bindings.extend(bind_functions(node))

    bindings.append(f'// End bindings for {header_path}')

    return bindings


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
        r'#include <pybind11/pybind11.h>',
        r'#include <pybind11/operators.h>',
    ]

    body = []

    print('Parsing header files...')
    index, translation_units = load_data(args.compilation_database,
                                         args.headers)

    print('Generating bindings...')
    bodies = [
        generate_bindings(header_path, translation_unit) for header_path,
        translation_unit in zip(args.headers, translation_units)
    ]

    prefix.extend(f"#include <{header_path.relative_to('include')}>"
                  for header_path in args.headers)
    prefix.extend([
        'namespace py = pybind11;', f'PYBIND11_MODULE({args.module_name}, m) {{'
    ])
    output = prefix + [line for body in bodies for line in body] + ['}']
    print(f'Outputting bindings to {args.output_file}')
    with open(args.output_file, 'w') as output_file:
        output_file.writelines([
            line for pair in zip(output, repeat('\n', len(output)))
            for line in pair
        ])
