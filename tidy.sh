#!/bin/bash

cd "${0%/*}"
source .ci/tidy-functions.sh

generate-compile-commands robowflex_library
fix-tidy
generate-compile-commands robowflex_ompl
fix-tidy
generate-compile-commands robowflex_movegroup
fix-tidy
generate-compile-commands robowflex_dart
fix-tidy
