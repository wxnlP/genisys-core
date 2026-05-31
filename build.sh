#!/bin/bash

cmake -Bbuild -S. -GNinja \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DBUILD_SIM_PROXY=OFF \
    -DBUILD_EXAMPLES=ON \
    $@


cmake --build build --target install
