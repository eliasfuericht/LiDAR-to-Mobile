# OutdoAR
LiDAR to mobile app

## Prerequisites

### Raspi-app:
- Ubuntu 24.04
- CMake 3.20
- gcc9/g++9

### Flutter-app:
- TBD

## Cloning & building raspi-app
```
git clone --recursive git@gitlab.cg.tuwien.ac.at:stef/outdoar.git
cd outdoar/raspi-app
mkdir build
cd build
cmake -DCMAKE_C_COMPILER=gcc-9 -DCMAKE_CXX_COMPILER=g++-9 ..
make -j1
```

Hint: Building the raspi-app is tested on Ubuntu 24.04 with g++ 9.5.0. Using more recent compilers like g++13 throws compiler errors. Using more than -j1 might drain the systems memory (tested with 8GB).