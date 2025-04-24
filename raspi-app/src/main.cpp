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

// livox lidar headers
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

// Open3D headers
#include "open3d/Open3D.h"

bool write_pointcloud_to_disk = false;

int counter = 0;

// receiver
std::string PHONE_IP;
int PHONE_PORT = 8888;

int SEND_SOCK;
struct sockaddr_in PHONE_ADDR = {};

static std::shared_ptr<open3d::geometry::PointCloud> s_previous_cloud = nullptr;

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

void RegisterPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& source,
  const std::shared_ptr<open3d::geometry::PointCloud>& target)
{
  if (!source || !target) return;

  // Estimate normals for the target point cloud
  target->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(20));

  // Optionally, orient normals consistently (can improve results)
  target->OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0.0, 0.0, 0.0)); // Adjust if needed

  open3d::pipelines::registration::RegistrationResult icp_result =
  open3d::pipelines::registration::RegistrationICP(
  *source,
  *target,
  0.1,
  Eigen::Matrix4d::Identity(),
  open3d::pipelines::registration::TransformationEstimationPointToPlane()
  );

  /*
  std::cout << "ICP Fitness: " << icp_result.fitness_ << std::endl;
  std::cout << "ICP RMSE: " << icp_result.inlier_rmse_ << std::endl;
  std::cout << "Transformation:\n" << icp_result.transformation_ << std::endl;
  */

  std::shared_ptr<open3d::geometry::PointCloud> aligned = std::make_shared<open3d::geometry::PointCloud>(*source);
  aligned->Transform(icp_result.transformation_);
}

void StorePointClouds()
{

}


void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) 
{
  static int cloud_counter = 0;
  // write the current data into a .ply file to "../pointclouds/cloudx.ply" where x is incremented every time
  if (data == nullptr || data->data_type != kLivoxLidarCartesianCoordinateHighData) return;

  LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;

  std::shared_ptr<open3d::geometry::PointCloud> current_cloud = std::make_shared<open3d::geometry::PointCloud>();

  // bool for writing to disk
  if (write_pointcloud_to_disk)
  { 
    std::ostringstream filename;
    filename << "../pointclouds/cloud" << cloud_counter++ << ".ply";
  
    std::ofstream ply_file(filename.str());
    if (ply_file.is_open()) {
        ply_file << "ply\n";
        ply_file << "format ascii 1.0\n";
        ply_file << "element vertex " << data->dot_num << "\n";
        ply_file << "property float x\n";
        ply_file << "property float y\n";
        ply_file << "property float z\n";
        ply_file << "end_header\n";
  
        for (uint32_t i = 0; i < data->dot_num; ++i) {
          ply_file << std::fixed << p_point_data[i].x << " " << p_point_data[i].y << " " << p_point_data[i].z << "\n";
        }
        ply_file.close();
    } else {
        std::cerr << "Failed to open file for writing: " << filename.str() << std::endl;
    }
  }

  for (uint32_t i = 0; i < data->dot_num; ++i) {
    double x = static_cast<double>(p_point_data[i].x) / 1000.0f;
    double y = static_cast<double>(p_point_data[i].y) / 1000.0f;
    double z = static_cast<double>(p_point_data[i].z) / 1000.0f;
    current_cloud->points_.emplace_back(x, y, z);
  }

  if (s_previous_cloud)
    RegisterPointCloud(current_cloud, s_previous_cloud);

  // save current cloud as previous for next iteration
  s_previous_cloud = current_cloud;

  uint32_t buffer_size = current_cloud->points_.size() * 3;
  std::vector<double> pos_buffer(buffer_size);

  memcpy(pos_buffer.data(), current_cloud->points_.data(), buffer_size * sizeof(double));

  ssize_t sent_bytes = sendto(
      SEND_SOCK,
      pos_buffer.data(),
      buffer_size * sizeof(double),
      0,
      (struct sockaddr*)&PHONE_ADDR,
      sizeof(PHONE_ADDR)
  );
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  } 
  //printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n", handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
} 
  
        
void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  //printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",status, handle, response->ret_code, response->error_key);
}

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  if (response == nullptr) {
    return;
  }
  //printf("RebootCallback, status:%u, handle:%u, ret_code:%u",status, handle, response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  //printf("LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",status, handle, response->ret_code, response->error_key);
    
  if (response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    printf("Query lidar internal info failed.\n");
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    return;
  }

  uint8_t host_point_ipaddr[4] {0};
  uint16_t host_point_port = 0;
  uint16_t lidar_point_port = 0;
  
  uint8_t host_imu_ipaddr[4] {0};
  uint16_t host_imu_data_port = 0;
  uint16_t lidar_imu_data_port = 0;
  
  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
    if (kv->key == kKeyLidarPointDataHostIpCfg) {
      memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    } else if (kv->key == kKeyLidarImuHostIpCfg) {
      memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }
  
  //printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n", host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

  //printf("Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n", host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);  
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    // printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  } 
  // printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  
  // set the work mode to kLivoxLidarNormal, namely start the lidar
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
  
  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;  
  //std::cout << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr) << ", push msg info: " << std::endl;
  //std::cout << info << std::endl;
  return;
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

    /*
    // dummy for testing
    for (uint32_t i = 0; i < 1000; i++)
    {
      uint32_t buffer_size = 8 * 3;
      double pos_buffer[buffer_size] = {
        -sin(i/100.0f), -sin(i/100.0f), -sin(i/100.0f),
        -sin(i/100.0f), -sin(i/100.0f),  sin(i/100.0f),
        -sin(i/100.0f),  sin(i/100.0f), -sin(i/100.0f),
        -sin(i/100.0f),  sin(i/100.0f),  sin(i/100.0f),
         sin(i/100.0f), -sin(i/100.0f), -sin(i/100.0f),
         sin(i/100.0f), -sin(i/100.0f),  sin(i/100.0f),
         sin(i/100.0f),  sin(i/100.0f), -sin(i/100.0f),
         sin(i/100.0f),  sin(i/100.0f),  sin(i/100.0f)
      };
      
      ssize_t sent_bytes = sendto(
        SEND_SOCK,
        pos_buffer,
        buffer_size * sizeof(double),
        0,
        (struct sockaddr*)&PHONE_ADDR,
        sizeof(PHONE_ADDR)
      );
      std::cout << i << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    */

    // init SDK
    if (!LivoxLidarSdkInit(path.c_str())) {
      printf("Livox Init Failed\n");
      LivoxLidarSdkUninit();
      return -1;
    }
    
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
    
    SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
    
    SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
    
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
    
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
