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

int i = 0;

struct LidarData
{
  int n = 1337;
};

void sendLidarData(const LidarData& data, int send_sock, struct sockaddr_in& phone_addr) {
  // Serialize LidarData (assuming it's a struct, adjust as needed)
  i++;
  std::string serializedData = std::to_string(i);

  // Send UDP packet
  ssize_t sent_bytes = sendto(send_sock, serializedData.c_str(), serializedData.size(), 0,
                              (struct sockaddr*)&phone_addr, sizeof(phone_addr));

  if (sent_bytes < 0) {
      std::cerr << "Failed to send LiDAR data\n";
  }
}

int main(int argc, const char *argv[]) {
  /*
  if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  */

  /*
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
  */

  // Mobile connection code here
  // 1. Create a UDP socket for sending data
  int send_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (send_sock < 0) {
      std::cerr << "Error creating UDP socket\n";
      return 1;
  }

  // 2. Set up the phone's UDP address
  struct sockaddr_in phone_addr = {};
  phone_addr.sin_family = AF_INET;
  phone_addr.sin_port = htons(PHONE_PORT);
  inet_pton(AF_INET, PHONE_IP, &phone_addr.sin_addr); // Convert IP

  // 3. Assume we already have LiDAR data in `LidarData data`
  while (true) {
      LidarData data;  // Assuming this gets updated continuously

      // 4. Send LiDAR data to the phone
      sendLidarData(data, send_sock, phone_addr);

      sleep(0.25);
  }

  LivoxLidarSdkUninit();
  printf("Livox Quick Start Demo End!\n");
  // Clean up (not needed for infinite loop)
  close(send_sock);
  return 0;
}