#!/bin/bash

if g++ -ggdb imgRec_v3.cpp -o imgRec_v3 `pkg-config --cflags --libs opencv`; then
    ./imgRec_v3
else
    echo "Compilation failed"
fi