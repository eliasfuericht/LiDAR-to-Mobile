# LiDAR-to-Mobile
Livox MID360 to Mobile Device Project

## Functionality
This project uses a mobile device (Android or iOS) for visualizing point cloud information in realtime. A Livox Mid360 LiDAR sensor streams point cloud data to a Raspberry Pi 5, which forwards the data via UDP to the mobile device for further processing and visualization. Every component can be powered using conventional powerbanks, which makes this setup versatile and extendable.

More information inside the individual project folders.

## Prerequisites

### raspi-app:
- Ubuntu 24.04
- CMake 3.20
- gcc9/g++9

### unity-mobile-app:
- Unity 6000.0.46f1
