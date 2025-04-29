# unity-mobile-app
Software that receives data via UDP and visualizes the pointcloud.

## Setup & Usage
This project was built using Windows 11, Unity 6000.0.46f1 and tested on Android 15 (Xiaomi HyperOS 2.0).

### Prerequisites
- Unity 6000.0.46f1

### Building
Open project with Unity Hub. Go to `File->Build Profiles`, select Android/iOS and click `Switch Platform`. Connect your phone via USB (tested on Android). Open `SampleScene` located in `Assets/Scenes/`. As soon as the scene is loaded, hit `CTRL+B` to build and select a location to store the final apk (somewhere on your pc, it doesn't really matter that much). After the build finished you will be prompted on your phone, hit yes to install the finished app.

### Usage
Open the app on your phone - data will be displayed as soon as it's received.


#### Notes & Additional Information
Currently the app displays `19968` points at a time, this is about `100ms` of data collected by the sensor. This number can be experimented with.
