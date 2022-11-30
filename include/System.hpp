#ifndef SYSTEM_H
#define SYSTEM_H

#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <vector>
#include <algorithm>

#include "config/config.hpp"
#include "DogStatus.hpp"
#include "Lidar.hpp"
#include "Control.hpp"

// LIDAR SDK
#include "rplidar.h"
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

// for display
#include "opencv2/opencv.hpp"

// for pcl 
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"

using namespace rp::standalone::rplidar;

class System {
    public:
        System(std::string input_path, std::string lidar_path, std::string arduino_path);
        void startProgram();
        void printSystemInfo();

    private:
        // for display
        cv::Mat m_camera_display;
        cv::Mat m_lidar_display;
        cv::Mat m_map_display;

        // for input 
        std::string m_input_path;
        bool is_use_camera = false;

        // for lidar 
        std::shared_ptr<Lidar> m_lidar;
        std::string m_lidar_path;

        // for arduino 
        std::shared_ptr<Control> m_control;
        std::string m_arduino_path;
        bool is_use_arduino = false;

        // for current data;
        std::shared_ptr<DogStatus> m_dog_status;

        // system flag 
        bool is_system_ready = false;

        // for thread
        std::thread camera_capture_thread;
        std::thread odometry_thread;
        std::thread control_thread;

        // thread method 
        static void cameraCaptureThread(std::string input_path, std::shared_ptr<DogStatus> dog_status);
        static void odometryThread(std::shared_ptr<Lidar> lidar, std::shared_ptr<DogStatus> dog_status);
        static void controlThread(std::shared_ptr<Control> control, std::shared_ptr<DogStatus> dog_status);
};

#endif
