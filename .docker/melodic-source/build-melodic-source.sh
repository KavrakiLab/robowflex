#!/bin/bash

cd "${0%/*}"
docker build -t robowflex/melodic-source -f Dockerfile `git rev-parse --show-toplevel`
