#!/bin/bash

# Note: this script requires the local ROS environment to be sourced.

function generate_compile_commands()
{
    mkdir .build-tmp
    pushd .build-tmp
    cmake ../$1/. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    commands=`realpath compile_commands.json`
    popd
}

function tidy()
{
	  run-clang-tidy -fix -header-filter=".*" -p $(dirname $commands)
}

cd "${0%/*}"
generate_compile_commands robowflex_library
tidy
