#!/bin/bash

cd "${0%/*}"
docker build -t robowflex/melodic -f Dockerfile `git rev-parse --show-toplevel`
