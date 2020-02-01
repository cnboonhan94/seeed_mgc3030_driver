#!/bin/sh
set -e 

PROJECT_ROOT=$(pwd)

echo "Building and Installing ncurses..."
sleep 2
cd $PROJECT_ROOT/external/ncurses-6.1
./configure
make
sudo make install

echo "Building and Installing WiringPi..."
sleep 2
cd $PROJECT_ROOT/external/WiringPi
./build

echo "Dependencies have been installed!"
