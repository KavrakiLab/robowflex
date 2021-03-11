#!/bin/bash

cd "${0%/*}"
docker build -t robowflex/noetic -f Dockerfile `git rev-parse --show-toplevel`
