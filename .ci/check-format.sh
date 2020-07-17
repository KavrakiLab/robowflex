#!/bin/bash
find .. -regex '.*\.\(cpp\|hpp\|cc\|cxx\)' -not -path "../.docs/*" -exec clang-format -style=file -output-replacements-xml {} \; | grep "<replacement "
RESULT="$?"

if [ "$RESULT" = "0" ]; then
    false
else
    true
fi
