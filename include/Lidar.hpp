#ifndef LIDAR_H
#define LIDAR_H
#include <iostream>
#include <vector>
#include <stdio.h>
#include <string>

#include "config/config.hpp"

// LIDAR SDK
#include "rplidar.h"
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

using namespace rp::standalone::rplidar;

class Lidar {
  public:
    Lidar(std::string lidar_path, int baud_rate);
    ~Lidar();
    size_t grabScanedLidarData(
        sl_lidar_response_measurement_node_hq_t* nodes,
        size_t counts);
    
  private:
    bool ctrl_c_pressed_ = false;
    IChannel* m_channel;
    ILidarDriver* m_drv;
    sl_result m_op_result;
    sl_lidar_response_device_info_t m_devinfo;

    bool checkSLAMTECLIDARHealth(ILidarDriver* drv);
};

#endif