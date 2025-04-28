# OutdoAR
LiDAR to mobile - a project by the Rendering and Modeling Group at TU Wien. 

## Functionality
This project uses a mobile device (Android or iOS) as the primary compute unit for real-time geometry processing. A Livox Mid360 LiDAR sensor streams point cloud data to a Raspberry Pi 5, which forwards the data via UDP to the mobile device for further processing and visualization. 

More information inside the individual project folders.

## Prerequisites

### raspi-app:
- Ubuntu 24.04
- CMake 3.20
- gcc9/g++9

### unity-mobile-app:
- Unity 6000.0.46f1