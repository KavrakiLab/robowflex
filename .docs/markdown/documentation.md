# Robowflex Documentation (a How-To) {#doc}

This document covers how to document code in Robowflex, as well as add additional tutorials such as this document here.

## Source Code Documentation

Documentation is automatically generated using [Doxygen](http://www.stack.nl/~dimitri/doxygen/) (see [documentation generation](@ref docgen) for more).
You can see the style of commenting throughout `robowflex_library`.
Any additional code added to `robowflex_library` or one of the optional modules should be fully documented.
Generally, all documentation is done in the header files, with little to no high-level documentation in the source files.
However, inline comments in methods are still encouraged, especially if there is some complex computation occuring.

### Class Documentation

A typical class declaration looks as follows:

\code{.cpp}
// There is extensive class forwarding in the library (declaration of shared pointer types). To have Doxygen 
// recognize these types, we define them directly above the class as follows:

/** \cond IGNORE */
ROBOWFLEX_CLASS_FORWARD(Example);
/** \endcond */

/** \class ExamplePtr
    \brief A shared pointer wrapper for Example. */

/** \class ExampleConstPtr
    \brief A const shared pointer wrapper for Example. */

/** \brief A short description of the class goes here.
 *
 *  A much longer description of the class and some of its intricacies goes in following paragraphs. We use a
 * line length of 110 characters total. Occasional, a class has a template parameter which is documented in
 * this block as well.
 */
 class Example
 {
 public:
     /** \brief A short description of a typedef to explain what it is.
      */
     typedef std::string String;
    
     /** \brief Constructor. A short description.
      *  \param[in] arg Description of what this argument is to the constructor.
      */
     Example(int arg);
    
     // Occasionally you have many functions in a class and would like to group them up functionally in the 
     // documentation. You can declare groups of functions with the following command, and they will appear 
     // grouped in the documentation:
     
     /** \name Subgroup
      *  \{ */
     
     /** \brief A function that modifies some input.
      *  \param[in] arg_in An input argument.
      *  \param[out] arg_out An argument that is modified or returned by the function.
      *  \return The return value of the function.
      */
     int doesStuff(int arg_in, int &arg_out);
  
     /** \brief Sometimes functions have templates!
      *  \param[in] arg An argument.
      *  \tparam T What does this template mean?
      */
     template <typename T>
     void doesStuffTemplate(T arg);
    
     // This closes the group.
     /** \} */
     
 private:
     int value_; ///< Member variables are documented like this.
 };
 \endcode

### Other Documentation

You should also document new namespaces you create.
This only needs to be done once, generally in a top-level header.

\code{.cpp}
/** \brief Contains all stuff relating to stuff.
 */
namespace stuff
{
}
\endcode

This enables documentation of top-level methods in that namespace, which should also be documented similar to class methods.

If you have some piece of code you want ignored by Doxygen, simple wrap it with the following:

\code{.cpp}
/** \cond IGNORE */

// Your ignored code goes here.

/** \endcond */
\endcode

### Python Documentation

There are a few Python scripts within the Robowflex code base, primarily within `robowflex_visualization`.
There is no consistent documentation for these methods yet, but every method you create should have at the very least a docstring as follows:

```py
def method()
  '''Here is some documentation about method.
  
  '''
  pass
```

## Adding New Tutorials / Pages

You can add new pages of high-level documentation like this one in two ways.
If you are documentation a package within Robowflex, if a `README.md` exists in the top-level of the project, it is automatically added.
See the `add_doc` macro in the `.docs`'s' `CMakeLists.txt` for where this is done.

Additionally, you can add a new markdown file (ending in a `*.md`) to the `.docs/markdown` folder.
Any markdown file in this directory is also automatically added when generating documentation.
