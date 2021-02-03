#!/bin/bash

if g++ -ggdb $1 -o imgRec_v3 `pkg-config --cflags --libs opencv`; then
    ./imgRec_v3
else
    echo "Compilation failed"
fi