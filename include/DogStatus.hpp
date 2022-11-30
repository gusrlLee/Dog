#ifndef DOG_STATUS_H
#define DOG_STATUS_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <string>
#include <queue>
#include <mutex>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"

#include "config/config.hpp"
#include "rplidar.h"
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"



class DogStatus {
    public:
        DogStatus();

        // for camera  
        cv::Mat getCurrentFrame();
        void setCurrentFrame(cv::Mat current_frame);

        // for system status 
        bool getSystemStatus();
        void setSystemStatus(bool flag);

        // for current_location    
        void setCurrentLocation(cv::Point2f current_location);
        cv::Point2f getCurrentLocation();

        // LiDAR data
        void setScanData(std::vector<sl_lidar_response_measurement_node_hq_t> current_lidar_scan_data, size_t count);
        std::vector<sl_lidar_response_measurement_node_hq_t> getScanData();

    private:
        cv::Mat m_current_frame;
        std::mutex current_frame_mutex;
        
        bool is_working;
        std::mutex is_working_mutex;

        cv::Point2f m_current_location;
        std::mutex current_location_mutex;

        std::vector<sl_lidar_response_measurement_node_hq_t> m_current_scan_data;
        std::mutex current_scan_data_mutex;
};

#endif // DOG_STATUS_H