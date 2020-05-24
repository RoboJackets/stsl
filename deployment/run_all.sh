#! /bin/bash
set -e
pushd cross_compilation
./run_cross_compile.sh
popd
pushd packaging
./run_packaging.sh
popd

