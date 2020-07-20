# Robowflex Code Style

Consistent, clean style in a code base leads to code that is more readable, debuggable, and maintainable.
Robowflex provides a prescribed style file (`.clang-format`), which enforces code style automatically.
To automatically format the code base, either use `clang-format` within your text editor, using the style file, or run the `format.sh` script.
Some of these conventions are captured in the `.clang-tidy` format file.
Use the `tidy.sh` script to automatically modernize your code base with `clang-tidy`.

However, there are some basic conventions that are also described in this file that must be considered when writing code for Robowflex.

## Naming Conventions

All files should be `lower_case`, separated by underscores.

All classes should be within the `robowflex` namespace.
Additional modules and specific components should also be placed in sub-namespaces.
All classes and structures should also be named using `CamelCase`.
All class methods and functions in general should be named with `camelCase`.
Endeavor to have all variable names be a single, descriptive word, lowercase.
If that is unavoidable, use `lower_case`.
Private/protected class member variables should end in an underscore.
Public class members should not end in an underscore.

## Header Style

Header files should always be placed in the folder `include/robowflex_*/`, with the extension `*.h`.
All headers should have header blockers in all-caps and an author description at the top:
```cpp
/* Author: Code Writer */

#ifndef ROBOWFLEX_<insert name here>_
#define ROBOWFLEX_<insert name here>_

... code goes here ...

#endif
```

Dependencies between header files should be minimized.
That is, if the exact declaration of an object is not needed in the header, and a forward declaration suffices, only the forward declaration should be used.
This is to minimize compile times and to untangle dependencies, which might get hairy as the code base scales.

To forward declare a class, use the class forwarding macro:
```cpp
#include <robowflex_library/class_forward.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Object);
    /** \endcond */
    
    ...
}  // namespace robowflex
```

Note the use of the Doxygen condition ignore around forward declarations.
This prevents doubling documentation of forward declarations.

Headers should never include code, to prevent issues of double declaration.
Template code can live in header files.

## Source Style

For single line scopes (e.g., after `if` or `for`), do not use brackets.
Always add a line of space after a scope.

Always use the `robowflex` namespace at the top of the file:
```cpp
using namespace robowflex;
```

You should separate class methods dedicated to different classes with a comment block:
```cpp
... OldClass code

///
/// NewClass
///

... NewClass code
```

## Other Conventions

- For compatibility reasons, `robowflex::RobotPose` should be used wherever a transformation is required.
- Most classes strive for `const`-correctness. Use `const` wherever possible. Moreover, use `const auto &`, in that order.
