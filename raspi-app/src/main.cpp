#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <sstream>

#include "common.hpp"
#include "LidarInterface.hpp"

// receiver
std::string PHONE_IP;
int PHONE_PORT = 8888;

int SEND_SOCK;
struct sockaddr_in PHONE_ADDR = {};

std::string getPhoneIP() {
  char buffer[128];
  std::string result = "";

  FILE* pipe = popen("ip route | grep default", "r");
  if (!pipe) return "ERROR";

  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    result += buffer;
  }
  pclose(pipe);

  std::istringstream iss(result);
  std::string word, via, ip;
  iss >> word >> via >> ip;

  return ip;
}

int i = 0;
void sendLidarData(int send_sock, struct sockaddr_in& phone_addr) {
  std::string serializedData = std::to_string(i++);

  // Send UDP packet
  ssize_t sent_bytes = sendto(send_sock, serializedData.c_str(), serializedData.size(), 0,
                              (struct sockaddr*)&phone_addr, sizeof(phone_addr));

  if (sent_bytes < 0) {
    std::cerr << "Failed to send LiDAR data\n";
  }
}


int main(int argc, const char *argv[]) {
  
  PHONE_IP = getPhoneIP();
  std::cout << "Detected Default Gateway(PHONE): " << PHONE_IP << std::endl;
  
  // Setting up UDP data transmission to mobile device
  SEND_SOCK = socket(AF_INET, SOCK_DGRAM, 0);
  if (SEND_SOCK < 0) {
    std::cerr << "Error creating UDP socket\n";
    return 1;
  }
  
  PHONE_ADDR.sin_family = AF_INET;
  PHONE_ADDR.sin_port = htons(PHONE_PORT);
  inet_pton(AF_INET, PHONE_IP.c_str(), &PHONE_ADDR.sin_addr);
  
  // Sending data
  std::cout << "Sending data to " << PHONE_IP << " via port " << PHONE_PORT << std::endl;
  
  // Setting up Livox Mid360
  // config file has to be in the same dir as executable
  const std::string path = "config.json";

  // init SDK
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

  // cleanup
  LivoxLidarSdkUninit();
  close(SEND_SOCK);
  return 0;
}