#!/bin/bash
# Hey, its not _my_ fault I didnt have time to use CMake...
g++ -g astra-goaldetect.cpp -I/opt/OpenNI-Linux-x64-2.2/Include -L/opt/OpenNI-Linux-x64-2.2/Redist -lOpenNI2 `pkg-config --cflags --libs opencv` -o astra-goaldetect
export LD_LIBRARY_PATH=/opt/OpenNI-Linux-x64-2.2/Redist/
if [[ $1 == "run" ]]; then
  ./astra-goaldetect
fi
