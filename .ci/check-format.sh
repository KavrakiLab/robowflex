#!/bin/bash

cd "${0%/*}"
find .. -regex '.*\.\(cpp\|h\)' -not -path "../.docs/*" -exec clang-format -style=file -output-replacements-xml {} \; | grep "<replacement "
RESULT="$?"

if [ "$RESULT" = "0" ]; then
    false
else
    true
fi
