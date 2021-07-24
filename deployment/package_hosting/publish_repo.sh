#! /usr/bin/env bash

set -e

distribution_name=focal
deb_tarball=../package_generation/debians.tar.gz

if [[ -z $(aptly repo list -config=aptly.config | grep stsl-release) ]]; then
    echo "stsl-release repo not found. Creating..."
    aptly repo create -config=aptly.config -distribution=$distribution_name -component=main stsl-release
fi


if [[ -d "./debians" ]]; then
    rm -rf ./debians/*
else
    mkdir ./debians
fi
tar -xf $deb_tarball -C ./debians

aptly repo add -config=aptly.config -force-replace stsl-release debians

snapshot_name="stsl-"$(date +"%Y%m%d%H%M")

aptly snapshot create -config=aptly.config $snapshot_name from repo stsl-release

if [[ -z $(aptly publish list -config=aptly.config | grep stsl-release) ]]; then
    aptly publish snapshot -config=aptly.config -gpg-key=90EF97AA7199324C $snapshot_name filesystem:stsl-release:
else
    aptly publish switch -config=aptly.config $distribution_name filesystem:stsl-release: $snapshot_name
fi

az storage blob upload-batch -d \$web -s ./published_repo --account-name stslaptstorage --auth-mode login 
