# for OSX and Linux
mkdir -p build
cd build
env CXX=/usr/bin/clang++ cmake ..
make -j 12
