#! /usr/bin/env bash

set -e

docker build --no-cache -t stsl_build -f Dockerfile ../..
docker create -ti --name stsl_build_temp stsl_build bash
docker cp stsl_build_temp:/usr/src/workspace/src/stsl/debians.tar.gz .
docker rm -f stsl_build_temp
