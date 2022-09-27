#!/bin/bash

cd "${0%/*}"
source .ci/tidy-functions.sh

# generate-compile-commands robowflex_moveit
# fix-tidy
# generate-compile-commands robowflex_dart
# fix-tidy
generate-compile-commands robowflex_moveit_scripts
fix-tidy
