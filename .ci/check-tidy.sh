#!/bin/bash

cd "${0%/*}"
source tidy-functions.sh

cd ..
generate-compile-commands robowflex_library
check-tidy
generate-compile-commands robowflex_ompl
check-tidy
generate-compile-commands robowflex_movegroup
check-tidy
generate-compile-commands robowflex_dart

