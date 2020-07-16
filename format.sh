#!/bin/bash
find .. -regex '.*\.\(cpp\|hpp\|cc\|cxx\)' -not -path "../.docs/*" -exec clang-format -style=file -i {} \;
