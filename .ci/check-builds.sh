#!/bin/bash

set -e
cd "${0%/*}"
pushd ../.docker
./kinetic/build-kinetic.sh
./melodic/build-melodic.sh
./melodic-source/build-melodic-source.sh
./noetic/build-noetic.sh
./tesseract/build-tesseract.sh
popd
