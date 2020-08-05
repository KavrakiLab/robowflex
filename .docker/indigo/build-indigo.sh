#!/bin/bash

cd "${0%/*}"
docker build -t robowflex/indigo -f Dockerfile `git rev-parse --show-toplevel`
