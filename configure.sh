#! /bin/sh
cd libcbdetect; cmake . -DCMAKE_BUILD_TYPE=Debug; cd ..; cmake -S . -B build/ -DCMAKE_BUILD_TYPE=Debug