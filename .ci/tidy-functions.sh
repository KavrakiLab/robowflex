#!/bin/bash

# Note: this script requires the local ROS environment to be sourced.
function generate-compile-commands()
{
    mkdir .build-tmp
    pushd .build-tmp
    CC="clang" CXX="clang++" cmake ../$1/. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    commands=`realpath compile_commands.json`
    popd
}

function fix-tidy()
{
	  run-clang-tidy -fix -header-filter=".*" -quiet -p .build-tmp
    rm -rf .build-tmp
}

function check-tidy()
{
	  run-clang-tidy -header-filter=".*" -quiet -p .build-tmp
    rm -rf .build-tmp
}

# Use include-what-you-use
function fix-iwyu()
{
    iwyu_tool.py -p .build-tmp 2> /tmp/iwyu.out
    fix_includes.py < /tmp/iwyu.out
    rm -rf .build-tmp
}
