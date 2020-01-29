#!/usr/bin/env bash

rm -rf build
mkdir build
pushd build

cmake ..
make

popd

