#!/bin/bash

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`
echo "Starting build..."

colcon build --symlink-install

popd > /dev/null