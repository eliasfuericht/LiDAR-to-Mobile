#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <iostream> 
#include <fstream>
#include <cmath>
#include <vector>

// livox lidar headers
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

// split up buffer into smaller ones if this number is exceeded
// leave space for udp headers and custom flag 
#define MAX_UDP_BUFFER_BYTES 65503

// flags for datatransfer
#define FLAG_POINT_DATA 0
#define FLAG_IMU_DATA 1

// variables for point data accumulation
// sensor sends 2083 packages @ 96 vertices per second (200000 points/s)
const int NUM_TOTAL_PACKAGES_PER_SECOND = 2083;
// NUM_POINTS_PER_PACKAGE: Number of points received from sensor per package (x,y,z)
const int NUM_POINTS_PER_PACKAGE = 96;

// NUM_PACKAGES: Number of point-data-packages (UDP packages) that get accumulated
int s_num_packages;
// TOTAL_NUM_POINTS: Number of points accumulated 
int s_total_num_points;
// BUFFER_SIZE: Size of buffer preallocated
int s_buffer_num_elements;
// keeps track of how many packages were received, reset when NUM_PACKAGES is reached
int32_t s_package_counter = 0;
// keeps track of index into buffer
int32_t s_buffer_index = 0;
// point data
std::vector<int32_t> s_pos_buffer;

// IMU data
LivoxLidarImuRawPoint s_imu_data;

// writing data to disk
bool s_write_data_to_disk = false;
int s_disc_data_counter = 0;

// udp receiver data
std::string s_phone_IP;
int s_phone_port = 8888;
struct sockaddr_in s_phone_adress = {};
int s_send_sock;

// helperfunction - getting ip of standardgateway (if connected with hotspot of phone -> will be phones ip)
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

// function to write point data to disk as .ply
void WriteToDisk()
{
  std::ostringstream filename;
  filename << "../data/accumulated_frames/" << s_num_packages << "_" << s_disc_data_counter++ << ".ply";

  std::ofstream ply_file(filename.str());
  if (ply_file.is_open()) 
  {
      ply_file << "ply\n";
      ply_file << "format ascii 1.0\n";
      ply_file << "element vertex " << s_total_num_points << "\n";
      ply_file << "property float x\n";
      ply_file << "property float y\n";
      ply_file << "property float z\n";
      ply_file << "end_header\n";

      for (uint32_t i = 0; i < s_buffer_num_elements; i+=3) 
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
  // calculate size of buffer to be sent
  int32_t current_udp_bytes = s_buffer_num_elements * sizeof(int32_t);

  ssize_t sent_bytes = -1;

  // buffer to hold flag + positionbuffer
  std::vector<int32_t> flag_prepended_buffer;

  // if current buffer is too big, subdivide buffer
  if (current_udp_bytes > MAX_UDP_BUFFER_BYTES)
  {
    // calculate how many buffers are needed
    int32_t num_split_buffers = std::ceil((float)current_udp_bytes / (float)MAX_UDP_BUFFER_BYTES);

    // increment if num_split_buffers is odd
    num_split_buffers += (num_split_buffers % 2);

    // calculate how many elements (one element is one component of a point, so x or y or z) per buffer
    int32_t num_elements_per_buffer = s_buffer_num_elements / num_split_buffers;

    // recalculate package size
    current_udp_bytes = num_elements_per_buffer * sizeof(int32_t);

    flag_prepended_buffer.resize(num_elements_per_buffer + 1);

    // prepend flag
    flag_prepended_buffer[0] = FLAG_POINT_DATA;

    for (int i = 0; i < num_split_buffers; i++)
    {
      // memcpy bufferpart
      std::memcpy(flag_prepended_buffer.data() + 1, s_pos_buffer.data() + i * num_elements_per_buffer, current_udp_bytes);
      sent_bytes = sendto(s_send_sock, flag_prepended_buffer.data(), flag_prepended_buffer.size() * sizeof(int), 0, (struct sockaddr*)&s_phone_adress, sizeof(s_phone_adress));
    }
  }
  else
  {
    flag_prepended_buffer.resize(s_buffer_num_elements + 1);

    flag_prepended_buffer[0] = FLAG_POINT_DATA;

    std::memcpy(flag_prepended_buffer.data() + 1, s_pos_buffer.data(), current_udp_bytes);
    sent_bytes = sendto(s_send_sock, flag_prepended_buffer.data(), flag_prepended_buffer.size() * sizeof(int), 0, (struct sockaddr*)&s_phone_adress, sizeof(s_phone_adress));
  }

  if (sent_bytes < 0)
  {
    std::cerr << "Failed to send point data to mobile\n";
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
    
    // fill buffer with elements
    for (uint32_t i = 0; i < num_points; i++) 
    {
      s_pos_buffer[s_buffer_index++] = p_point_data[i].x;
      s_pos_buffer[s_buffer_index++] = p_point_data[i].y;
      s_pos_buffer[s_buffer_index++] = p_point_data[i].z;
    }

    // send data as soon as s_num_packages is reached
    if (s_package_counter % s_num_packages == 0)
    {
      // reset index into buffer
      s_buffer_index = 0;

      SendDataToMobile();
      
      if (s_write_data_to_disk)
      {
        WriteToDisk();
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
    s_imu_data = *imu_data;

    std::vector<float> flag_prepended_buffer;

    // hardcoded since IMU layout isn't dynamic
    flag_prepended_buffer.resize(6 + 1);

    // set flag
    *reinterpret_cast<int*>(&flag_prepended_buffer[0]) = FLAG_IMU_DATA;

    flag_prepended_buffer[1] = s_imu_data.acc_x;
    flag_prepended_buffer[2] = s_imu_data.acc_y;
    flag_prepended_buffer[3] = s_imu_data.acc_z;
    flag_prepended_buffer[4] = s_imu_data.gyro_x;
    flag_prepended_buffer[5] = s_imu_data.gyro_y;
    flag_prepended_buffer[6] = s_imu_data.gyro_z;

    ssize_t sent_bytes = sendto(s_send_sock, flag_prepended_buffer.data(), flag_prepended_buffer.size() * sizeof(float), 0, (struct sockaddr*)&s_phone_adress, sizeof(s_phone_adress));
  
    if (sent_bytes < 0)
    {
      std::cerr << "Failed to send IMU data to mobile\n";
    }
  }
}

// helper function for input parameters
bool is_number(const std::string& s) 
{
  for (char const& ch : s) 
  {
    if (!std::isdigit(ch)) return false;
  }
  return true;
}

// Calculate data per package if accumulation is active
void SetupTransfereRate(float update_rate)
{
  // set to insta send packages if 0
  if (update_rate == 0)
  {
    s_num_packages = 1;
  }
  else // calc number of packages to accumulate
  {
    float data_rate = update_rate / 1000.0f;
  
    // setting up buffer that holds data for forwarding to mobile
    s_num_packages = (int)(NUM_TOTAL_PACKAGES_PER_SECOND * data_rate);
  }

  s_total_num_points = NUM_POINTS_PER_PACKAGE * s_num_packages;
  s_buffer_num_elements = s_total_num_points * 3;
  s_pos_buffer.resize(s_buffer_num_elements);

  if (s_num_packages > 1)
  {
    std::cout << s_num_packages << " packages will be cumulated and a total of " 
              << s_total_num_points << " will be sent every " << update_rate << " milliseconds." << std::endl;
  }
  else
  {
    std::cout << "forwarding packages instantly without accumulation." << std::endl;
  }
}

// Setting up UDP data transmission to mobile device
bool SetupUDP()
{
  s_phone_IP = getPhoneIP();
  
  s_send_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (s_send_sock < 0) 
  {
    std::cerr << "Error creating UDP socket\n";
    return false;
  }
  
  s_phone_adress.sin_family = AF_INET;
  s_phone_adress.sin_port = htons(s_phone_port);
  inet_pton(AF_INET, s_phone_IP.c_str(), &s_phone_adress.sin_addr);
  
  std::cout << "Sending data to " << s_phone_IP << " via port " << s_phone_port << std::endl;
  return true;
}

int main(int argc, const char *argv[]) 
{
  if (argc != 2 || !is_number(argv[1]) || std::atof(argv[1]) < 0) 
  {
    std::cout << "Params Invalid, please specify the data rate in milliseconds (integer)." << std::endl;
    return -1;
  }
  
  SetupTransfereRate(std::atof(argv[1]));

  if (!SetupUDP())
  {
    std::cerr << "Error setting up UDP\n";
    return 1;
  }

  // Setting up Livox Mid360
  // init SDK
  // config file has to be in the same dir as executable
  if (!LivoxLidarSdkInit("receiver-config.json")) {
    std::cout << "Livox Init Failed\n" << std::endl;
    LivoxLidarSdkUninit();
    return -1;
  }
  
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);

  SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);

  // lets program run indefinitely until it is terminated from outside
  while(true)
  {
    sleep(300);
  }
  
  // cleanup
  LivoxLidarSdkUninit();
  close(s_send_sock);
  return 0;
}
