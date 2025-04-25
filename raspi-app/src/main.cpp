#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream> 
#include <fstream>

// livox lidar headers
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

bool s_write_pointcloud_to_disk = true;
int s_point_data_counter = 0;

// receiver
std::string PHONE_IP;
int PHONE_PORT = 8888;

int SEND_SOCK;
struct sockaddr_in PHONE_ADDR = {};

LivoxLidarImuRawPoint current_IMU_data;

// accumulate 200 packages (around 100ms of scanning)
#define NUM_PACKAGES 200
#define NUM_POINTS_PER_PACKAGE 96
#define NUM_POINTS (NUM_POINTS_PER_PACKAGE * NUM_PACKAGES)
#define BUFFER_SIZE (NUM_POINTS * 3)
int32_t s_pos_buffer[BUFFER_SIZE];

int32_t s_counter = 0;

int32_t s_buffer_index = 0;

std::string getPhoneIP() {
  char buffer[128];
  std::string result = "";

  FILE* pipe = popen("ip route | grep '^default' | grep 'wlan0'", "r");
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

void WriteToDisk()
{
  std::ostringstream filename;
  filename << "../pointclouds/cloud" << s_point_data_counter++ << ".ply";

  std::ofstream ply_file(filename.str());
  if (ply_file.is_open()) 
  {
      ply_file << "ply\n";
      ply_file << "format ascii 1.0\n";
      ply_file << "element vertex " << NUM_POINTS << "\n";
      ply_file << "property float x\n";
      ply_file << "property float y\n";
      ply_file << "property float z\n";
      ply_file << "end_header\n";

      for (uint32_t i = 0; i < BUFFER_SIZE; i+=3) 
      {
        ply_file << std::fixed << s_pos_buffer[i] << " " << s_pos_buffer[i+1] << " " << s_pos_buffer[i+2] << "\n";
      }
      ply_file.close();
  } 
  else 
  {
      std::cerr << "Failed to open file for writing: " << filename.str() << std::endl;
  }
}

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) 
{
  if (data == nullptr) 
  {
    return;
  }
    
  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) 
  {
    s_counter++;
    int32_t dot_num = data->dot_num;
    
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    
    for (uint32_t i = 0; i < dot_num; i++) 
    {
      s_pos_buffer[s_buffer_index++] = p_point_data[i].x;
      s_pos_buffer[s_buffer_index++] = p_point_data[i].y;
      s_pos_buffer[s_buffer_index++] = p_point_data[i].z;
    }

    if (s_counter % NUM_PACKAGES == 0)
    {
      if (s_write_pointcloud_to_disk)
      { 
        WriteToDisk();
      }
      s_buffer_index = 0;

      // register now?

      //send pos_buffer
      ssize_t sent_bytes = sendto(SEND_SOCK, s_pos_buffer, BUFFER_SIZE * sizeof(int32_t), 0, (struct sockaddr*)&PHONE_ADDR, sizeof(PHONE_ADDR));
    }
  }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) 
{
  if (data == nullptr) 
  {
    return;
  }
    
  if (data->data_type == kLivoxLidarImuData) 
  {
    LivoxLidarImuRawPoint *imu_data = (LivoxLidarImuRawPoint *)data->data;
  }
}

int main(int argc, const char *argv[]) 
{
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
    
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);

    SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);

    // think about how to terminate the program from the phone
    while(true)
    {
      sleep(300);
    }
    
    // cleanup
    LivoxLidarSdkUninit();
    close(SEND_SOCK);
    return 0;
  }
