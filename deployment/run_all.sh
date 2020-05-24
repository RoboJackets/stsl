#! /bin/bash
set -e
./cross_compilation/run_cross_compilation.sh
./packaging/run_packaging.sh
