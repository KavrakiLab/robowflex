# Robowflex Documentation Generation {#docgen}
Documentation is generated with [doxygen](https://www.doxygen.nl/index.html).

## Dependencies
Install all necessary dependencies to build documentation:
```sh
sudo apt install doxygen cmake graphviz
```

## Building Documentation
Starting in this folder (`.docs`):
```sh
mkdir build
cd build
cmake ..
make
```
This will create and populate the folder `doc/html` with the generated documentation website.

## Automatic Generation
Continuous integration through [Travis CI](https://travis-ci.org/) is used to automatically generate the [Github pages website](https://kavrakilab.github.io/robowflex/index.html).
The documentation is automatically pushed to the `gh-pages` branch.
