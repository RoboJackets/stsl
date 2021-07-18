#! /usr/bin/env bash

set -e


# https://assafmo.github.io/2019/05/02/ppa-repo-hosted-on-github.html
# https://robojackets.github.io/stsl is now an apt repo!!! Yay!

# To get access to the packages:
# curl -s --compressed "https://robojackets.github.io/stsl/KEY.gpg" | sudo apt-get add -
# sudo apt-add-repository "deb https://robojackets.github.io/stsl ./"
# 
# To confirm it works, this command should show some package names:
# apt-cache search ros-foxy-stsl
