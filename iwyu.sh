#!/bin/bash

cd "${0%/*}"
source .ci/tidy-functions.sh

generate-compile-commands robowflex_library
fix-iwyu
generate-compile-commands robowflex_ompl
fix-iwyu
generate-compile-commands robowflex_movegroup
fix-iwyu
generate-compile-commands robowflex_dart
fix-iwyu
