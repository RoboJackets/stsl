#! /bin/bash

set -e

tar -xzf stsl_package.tar.gz

read -p 'Robot number: ' ROBOT_NUMBER

sed "s/<ROBOT_NUMBER>/$ROBOT_NUMBER/g" robojacketsstsl.service.in | sudo tee /etc/systemd/system/robojacketsstsl.service

sudo chmod 644 /etc/systemd/system/robojacketsstsl.service

sudo systemctl enable robojacketsstsl
