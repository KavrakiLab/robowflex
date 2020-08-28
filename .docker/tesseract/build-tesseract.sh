#!/bin/bash

cd "${0%/*}"
docker build -t robowflex/tesseract -f Dockerfile `git rev-parse --show-toplevel`
