#!/bin/bash

cd "${0%/*}"
docker build -t robowflex/kinetic -f Dockerfile `git rev-parse --show-toplevel`
