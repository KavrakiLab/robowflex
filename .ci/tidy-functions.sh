#!/bin/bash

# Note: this script requires the local ROS environment to be sourced.
function generate-compile-commands()
{
    mkdir .build-tmp
    pushd .build-tmp
    cmake ../$1/. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
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

# Use include-what-you-use to find missing / unnecessary includes
function get-iwyu()
{
    iwyu_tool.py -p .build-tmp | tee -a iwyu.out
    rm -rf .build-tmp
}
