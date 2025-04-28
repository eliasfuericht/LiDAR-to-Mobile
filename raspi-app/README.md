# raspi-app
Software that receives data from Livox MID360, optionally writes data to disk, and forwards it to phone.

## Setup & Usage
This software is intended (and tested) to run on Raspberry Pi 5 8GB with Ubuntu Server 24.04.2 LTS (64-bit).

### Prerequisites
- Ubuntu 24.04
- CMake 3.20
- gcc9/g++9

### Building
```
cd outdoar/raspi-app
mkdir build
cd build
cmake -DCMAKE_C_COMPILER=gcc-9 -DCMAKE_CXX_COMPILER=g++-9 ..
make -j1
```

### Usage
- Edit the `01-livox-receiver-network-config.yaml` in `outdoar/raspi-app` and fill in your phone's hotspot credentials. 
- Copy the config file to the netplan folder: `sudo cp ./01-livox-receiver-network-config.yaml /etc/netplan/`
- From the terminal call `sudo netplan apply`, this will change the raspi's network config so it can receive data from the Livox MID360.
- The raspi should now be connected to your phone's hotspot.
- Plug in the Livox MID360 via ethernet into the Raspberry, and start the MID360.
- `cd build`
- Execute the program: `./OutDoARServer 0`.
- Data is now sent to your phone and can be inspected when running the unity application `outdoar/unity-mobile-app` on your phone.

#### Notes & Additional Information
When executing the program it expects an integer as a first argument, this integer sets the rate in milliseconds at which data is forwarded to the phone. `0` meaning instant, so every package that is received from the sensor gets forwarded instantaneously. Putting a value `n > 0` will accumulate packages for the specified time. This also applies when writing data to disk. `0` -> writes file for every package received. `n` -> accumulates for n milliseconds and writes data to disk.

The sensor sends `2083 packages per second with 96 points per package`, resulting in a rate of `199986 points/s`. The sensor uses non-repetitive scanning, which is the reason I implemented the accumulation of packages, since the scene gets more and more detail, the longer information is accumulated, see [Livox MID360 Docs](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf) for more information, especially section [Product Characteristics].