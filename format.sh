#!/bin/bash

cd "${0%/*}"
find . -regex '.*\.\(cpp\|h\)' -not -path ".docs/*" -exec clang-format -style=file -i {} \;
