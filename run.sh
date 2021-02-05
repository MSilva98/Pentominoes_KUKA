#!/bin/bash

if g++ -ggdb $1 -o imgRec `pkg-config --cflags --libs opencv`; then
    ./imgRec
else
    echo "Compilation failed"
fi