#!/bin/bash

cmake -Bbuild -S. \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DGENISYS_CORE_BUILD_EXAMPLES=ON \
    -DDMDRIVER_BUILD_WITH_EXAMPLE=ON \
    $@


cmake --build build --target install --parallel $(nproc)
