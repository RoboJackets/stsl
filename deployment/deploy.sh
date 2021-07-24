#! /usr/bin/env bash

set -e

pushd package_generation
./generate_debs.sh
popd

pushd package_hosting
./publish_repo.sh
popd

echo "Deployment complete!"
