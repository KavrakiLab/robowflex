#!/bin/bash

cd "${0%/*}"
find . -regex '.*\.\(cpp\|h\)' -not -path ".docs/*" -exec clang-tidy -fix -fix-errors {} -- -std=c++11 \;
