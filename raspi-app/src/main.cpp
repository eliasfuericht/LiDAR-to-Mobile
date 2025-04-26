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
#include <cmath>
#include <vector>

// livox lidar headers
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

// variables for point data accumulation
// sensor sends 2083 packages @ 96 vertices per second (200000 points/s)
#define TOTAL_PACKAGES_PER_SECOND 2083
// how fast lidardata is sent to mobile in ms(faster = more responsive but less detail)
// 208 ~= 100ms of frames (vertical fov also covered)
#define UPDATE_SPEED 10
// NUM_PACKAGES: Number of point-data-packages (UDP packages) that get accumulated
#define NUM_PACKAGES (TOTAL_PACKAGES_PER_SECOND / UPDATE_SPEED)
// NUM_POINTS_PER_PACKAGE: Number of points received from sensor per package (x,y,z)
#define NUM_POINTS_PER_PACKAGE 96
// TOTAL_NUM_POINTS: Number of points accumulated 
#define TOTAL_NUM_POINTS (NUM_POINTS_PER_PACKAGE * NUM_PACKAGES)
// BUFFER_SIZE: Size of buffer preallocated
#define BUFFER_NUM_ELEMENTS (TOTAL_NUM_POINTS * 3)
// keeps track of how many packages were received, reset when NUM_PACKAGES is reached
int32_t s_package_counter = 0;

// split up buffer into smaller ones if this number is reached
#define MAX_UDP_BUFFER_BYTES 65507
// Holds accumulated points
std::array<int32_t, BUFFER_NUM_ELEMENTS> s_pos_buffer;
// keeps track of index into buffer
int32_t s_buffer_index = 0;

// IMU data
LivoxLidarImuRawPoint s_current_imu_data;

// writing data to disk
bool s_write_pointcloud_to_disk = false;
int s_disc_data_counter = 0;

// udp receiver data
std::string PHONE_IP;
int PHONE_PORT = 8888;
struct sockaddr_in PHONE_ADDR = {};
int SEND_SOCK;

int counter = 0;

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
  filename << "../data/accumulated_frames/" << NUM_PACKAGES << "_" << s_disc_data_counter++ << ".ply";

  std::ofstream ply_file(filename.str());
  if (ply_file.is_open()) 
  {
      ply_file << "ply\n";
      ply_file << "format ascii 1.0\n";
      ply_file << "element vertex " << TOTAL_NUM_POINTS << "\n";
      ply_file << "property float x\n";
      ply_file << "property float y\n";
      ply_file << "property float z\n";
      ply_file << "end_header\n";

      for (uint32_t i = 0; i < BUFFER_NUM_ELEMENTS; i+=3) 
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

void SendDataToMobile()
{
  int32_t current_udp_bytes = BUFFER_NUM_ELEMENTS * sizeof(int32_t);

  ssize_t sent_bytes = -1;

  // if current buffer is too big, subdivide buffer
  if (current_udp_bytes > MAX_UDP_BUFFER_BYTES)
  {
    // calculate how many buffers are needed
    int32_t num_split_buffers = std::ceil((float)current_udp_bytes / (float)MAX_UDP_BUFFER_BYTES);

    // calculate how many elements (one element is one component of a point, so x or y or z) per buffer
    int32_t num_elements_per_buffer = BUFFER_NUM_ELEMENTS / num_split_buffers;

    // recalculate package size
    current_udp_bytes = num_elements_per_buffer * sizeof(int32_t);

    for (int i = 0; i < num_split_buffers; i++)
    {
      sent_bytes = sendto(SEND_SOCK, s_pos_buffer.data() + i * num_elements_per_buffer, current_udp_bytes, 0, (struct sockaddr*)&PHONE_ADDR, sizeof(PHONE_ADDR));
    }
    counter++;
  }
  else
  {

    sent_bytes = sendto(SEND_SOCK, s_pos_buffer.data(), current_udp_bytes, 0, (struct sockaddr*)&PHONE_ADDR, sizeof(PHONE_ADDR));
  }

  if (sent_bytes < 0)
  {
    std::cerr << "Failed to send data to mobile\n";
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
    s_package_counter++;

    int32_t num_points = data->dot_num;
    
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;

    // integrate s_current_imu_data here somewhere?
    
    // fill buffer with elements
    for (uint32_t i = 0; i < num_points; i++) 
    {
      s_pos_buffer[s_buffer_index++] = p_point_data[i].x;
      s_pos_buffer[s_buffer_index++] = p_point_data[i].y;
      s_pos_buffer[s_buffer_index++] = p_point_data[i].z;
    }

    // send data as soon as NUM_PACKAGES is reached
    if (s_package_counter % NUM_PACKAGES == 0)
    {
      // reset index into buffer
      s_buffer_index = 0;

      SendDataToMobile();
      
      if (s_write_pointcloud_to_disk)
      {
        WriteToDisk();
        std::cout << "stop" << std::endl;
      }
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
    s_current_imu_data = *imu_data;
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
