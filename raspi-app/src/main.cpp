#include "common.hpp"
#include "LidarInterface.hpp"

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

// Define phone details
#define PHONE_IP "192.168.247.216" // Change this to match your phone’s IP
#define PHONE_PORT 8888         // The port where the phone listens for UDP data

int main(int argc, const char *argv[]) {
  std::cout << "hello" << std::endl;
  /*
  if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  */
  const std::string path = "config.json";
  
  // REQUIRED, to init Livox SDK2
  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }
  
  // REQUIRED, to get point cloud data via 'PointCloudCallback'
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  
  // OPTIONAL, to get imu data via 'ImuDataCallback'
  // some lidar types DO NOT contain an imu component
  SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  
  SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  
  // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  // Mobile connection code here


  
  #ifdef WIN32
  Sleep(300000);
  #else
  sleep(300);
  #endif
  LivoxLidarSdkUninit();
  printf("Livox Quick Start Demo End!\n");
  return 0;
}