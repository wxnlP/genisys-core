#!/bin/bash

cmake -Bbuild -S. \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DSDK_BUILD_WITH_EXAMPLES=ON \
    -DDMDRIVER_BUILD_WITH_EXAMPLE=ON \
    $@


cmake --build build --target install --parallel $(nproc)
