#!/bin/bash

cmake -Bbuild -S. -DCMAKE_INSTALL_PREFIX=./build/install
cmake --build build --target install
