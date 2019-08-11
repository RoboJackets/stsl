#!/usr/bin/env bash

echo 'Resetting the software training workspace!'
cd /home/debian/software-training/projects
git checkout .
git clean -fdx

