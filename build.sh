#!/bin/bash

cmake -Bbuild -S. -GNinja \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    # -DBUILD_DEFAULT_SERIAL=ON \
    # -DBUILD_EXAMPLES=ON \
    $@


cmake --build build --target install
