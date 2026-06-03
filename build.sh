#!/bin/bash

cmake -Bbuild -S. -GNinja \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DBUILD_SIM_PROXY=OFF \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_DEFAULT_SERIAL=ON \
    -DDMDRIVER_BUILD_WITH_EXAMPLE=ON \
    $@


cmake --build build --target install --parallel 2
