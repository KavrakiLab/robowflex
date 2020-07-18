#!/bin/bash
mkdir .docs/build
pushd .docs/build
cmake ..
make
popd
