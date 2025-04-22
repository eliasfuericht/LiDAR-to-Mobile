# OutdoAR
LiDAR to mobile - a project by the Rendering and Modeling Group at TU Wien. 

## Functionality
This project uses a mobile device (Android or iOS) as the primary compute unit for real-time geometry processing. A Livox Mid360 LiDAR sensor streams point cloud data to a Raspberry Pi 5, which registers and forwards the data via UDP to the mobile device for further processing and visualization.

## Prerequisites

### raspi-app:
- Ubuntu 24.04
- CMake 3.20
- gcc9/g++9
- bunch of packages (see setup.sh)

### unity-mobile-app:
- Unity 6000.0.46f1

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