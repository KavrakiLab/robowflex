#!/bin/bash

cd "${0%/*}"
source .ci/tidy-functions.sh

generate-compile-commands robowflex_library
get-iwyu
generate-compile-commands robowflex_ompl
get-iwyu
generate-compile-commands robowflex_movegroup
get-iwyu
generate-compile-commands robowflex_dart
get-iwyu
