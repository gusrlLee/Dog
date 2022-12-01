#include "System.hpp"
#include <ctime>

using namespace rp::standalone::rplidar;

System::System(std::string input_path, std::string lidar_path, std::string arduino_path) {

    // for camera init 
    printf("[SYSTEM]: Initialization Camera....");
    if ( !input_path.empty() ) {
        m_input_path = input_path;
        is_use_camera = true;     
        printf("\t[OK]\n");  
    }
    else {
        is_use_camera = false;
        printf("\t[NOT USE]\n"); 
    }

    // LiDAR init 
    printf("[SYSTEM]: Initialization LiDAR....");
    m_lidar = std::make_shared<Lidar>(lidar_path.c_str(), LIDAR_BAUDRATE);
    m_lidar_path = lidar_path;
    printf("\t[OK]\n");

    // DogStatus init 
    printf("[SYSTEM]: Initialization Dog Status....");
    m_dog_status = std::make_shared<DogStatus>();
    printf("\t[OK]\n");

    // control init
    printf("[SYSTEM]: Initialization Control ....");
    if ( !arduino_path.empty() ) {
        m_control = std::make_shared<Control>(arduino_path.c_str(), UART_BAUDRATE);
        m_arduino_path = arduino_path;
        is_use_arduino = true;
        printf("\t[OK]\n");
    }
    else {
        is_use_arduino = false;
        printf("\t[NOT USE]\n"); 
    }

    // check start flag 
    this->printSystemInfo();
    is_system_ready = true;
    m_dog_status->setSystemStatus(true);

    // thread execute
    odometry_thread = std::thread(&System::odometryThread, m_lidar, m_dog_status);

    if ( is_use_camera )
        camera_capture_thread = std::thread(&System::cameraCaptureThread, m_input_path, m_dog_status);

    if ( is_use_arduino ) 
        control_thread = std::thread(&System::controlThread, m_control,m_dog_status);
}

void System::printSystemInfo() {
    std::cout << "==========================System Information===========================" << std::endl;
    std::cout << "AUTO DOG SYSTEM v1.0.0" << std::endl;
    if ( !m_input_path.empty() )
        std::cout << "INPUT PATH = " << m_input_path << std::endl;
    else 
        std::cout << "INPUT PATH = [NOT USE]" << std::endl;

    std::cout << "LIDAR PATH = " << m_lidar_path << std::endl;
    std::cout << "LIDAR BAUDRATE = " << LIDAR_BAUDRATE << std::endl;

    if ( !m_arduino_path.empty() ) {
        std::cout << "ARDUINO PATH = " << m_arduino_path << std::endl;
        std::cout << "ARDUINO BAUDRATE = " << UART_BAUDRATE << std::endl; 
    } 
    else {
        std::cout << "ARDUINO PATH = [NOT USE]"<< std::endl;
        std::cout << "ARDUINO BAUDRATE = [NOT USE]" << std::endl; 
    }
    std::cout << "Auto System Start..." << std::endl;
    std::cout << "=======================================================================" << std::endl;
    std::cout << "If you want exit system, Press ESC key" << std::endl;
}

void System::startProgram() {

    float theta = 0;
    float object_distance = 0;
    cv::Point center = cv::Point(500, 500);
    float rotate = 90;

    while (1) {
        if (!m_dog_status->getSystemStatus()) {
            printf("[EXIT]\n");
            break;
        }
        // init display
        cv::Mat lidar_display = cv::Mat::zeros(cv::Size(1000, 1000), CV_8UC3);
        cv::Mat map_display = cv::Mat::zeros(cv::Size(1000, 1000), CV_8UC3);

        // get dataset
        std::vector<sl_lidar_response_measurement_node_hq_t> current_scan_data;
        current_scan_data = m_dog_status->getScanData();
        cv::Point2f current_location = m_dog_status->getCurrentLocation();

        size_t count = current_scan_data.size();

        cv::circle(map_display, cv::Point((int)current_location.x, (int)current_location.y), 5, (255, 0, 0), 5, cv::FILLED);

        for (int pos = 0; pos < (int)count; ++pos) {
            theta = current_scan_data[pos].angle_z_q14 * 90.f / 16384.f;
            object_distance = current_scan_data[pos].dist_mm_q2 / 4.0f;

            if (object_distance > 0) {
                if (object_distance < LIDAR_MAX_DISTANCE) {
                    int x = object_distance * cos(theta * (CV_PI/180));
                    int y = object_distance * sin(theta * (CV_PI/180));
                    float rx = x * cos(rotate * (CV_PI/180)) - y * sin(rotate * (CV_PI/180));
                    float ry = x * sin(rotate * (CV_PI/180)) + y * cos(rotate * (CV_PI/180));

                    // for map 
                    int xx = rx * MAP_PIXEL_RATIO;
                    int yy = ry * MAP_PIXEL_RATIO;
                    cv::circle(map_display, cv::Point(xx + current_location.x, yy + current_location.y), 1, cv::Scalar(0, 0, 255), 1, cv::FILLED);

                    // for lidar 
                    xx = rx * LIDAR_PIXEL_RATIO;
                    yy = ry * LIDAR_PIXEL_RATIO;
                    cv::circle(lidar_display, cv::Point(xx + center.x, yy + center.y), 1, cv::Scalar(0, 0, 255), 1, cv::FILLED);
                    cv::line(lidar_display, cv::Point(center.x, center.y), cv::Point(xx + center.x, yy + center.y), cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
                    
                }  

                else {
                    int x = LIDAR_MAX_DISTANCE * cos(theta * (CV_PI/180)); 
                    int y = LIDAR_MAX_DISTANCE * sin(theta * (CV_PI/180));
                    float rx = x * cos(rotate * (CV_PI/180)) - y * sin(rotate * (CV_PI/180));
                    float ry = x * sin(rotate * (CV_PI/180)) + y * cos(rotate * (CV_PI/180));

                    // for map 
                    int xx = rx * MAP_PIXEL_RATIO;
                    int yy = ry * MAP_PIXEL_RATIO;
                    cv::circle(map_display, cv::Point(xx + current_location.x, yy + current_location.y), 1, cv::Scalar(0, 0, 255), 1, cv::FILLED);

                    // for lidar 
                    xx = rx * LIDAR_PIXEL_RATIO;
                    yy = ry * LIDAR_PIXEL_RATIO;
                    cv::circle(lidar_display, cv::Point(xx + center.x, yy + center.y), 1, cv::Scalar(0, 0, 255), 1, cv::FILLED);
                    cv::line(lidar_display, cv::Point(center.x, center.y), cv::Point(xx + center.x, yy + center.y), cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
                }
            }
        }
        // cv::flip(map_display, map_display, 1);


        // display
        cv::imshow("LIDAR", lidar_display);
        cv::imshow("Map", map_display);

        if ( is_use_camera ) {
            cv::Mat current_frame = m_dog_status->getCurrentFrame();
            if (current_frame.empty()) continue;
            cv::imshow("Frame", current_frame);
        }
        int key = cv::waitKey(1);
        if (key == 27) {
            m_dog_status->setSystemStatus(false);
            break;
        }
    }

    // wait thread 
    odometry_thread.join();
    if ( is_use_camera )
        camera_capture_thread.join();

    if ( is_use_arduino ) 
        control_thread.join();
}

void System::cameraCaptureThread(std::string input_path, std::shared_ptr<DogStatus> dog_status) {
    cv::VideoCapture cap(input_path);
    cv::Mat current_frame;

    if (!cap.isOpened()) {
        printf("[ERROR] Check Your Video or Camrea path!!!\n");
        exit(1);
    }

    while (1) {
        if (!dog_status->getSystemStatus()) {
            printf("[SYSTEM] Close Your Camera!\n");
            cap.release();
            break;
        }
        cap >> current_frame;
        if (current_frame.empty()) {
            continue;
        }
        cv::flip(current_frame, current_frame, 0);

        dog_status->setCurrentFrame(current_frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void System::odometryThread(std::shared_ptr<Lidar> lidar, std::shared_ptr<DogStatus> dog_status) {
    // for icp algorithm
    std::vector<cv::Point2f> prev_scan_data;
    std::vector<cv::Point2f> curr_scan_data;
    cv::Point2f current_location = cv::Point2f(500, 500);
    bool is_ready = false;
    float rotate = 90;

    while (1) {
        if (!dog_status->getSystemStatus()) {
            printf("[SYSTEM] Exit Odometry Thread\n");
            break;
        }

        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);
        count = lidar->grabScanedLidarData(nodes, count);
        std::vector<sl_lidar_response_measurement_node_hq_t> current_scan_data(std::begin(nodes), std::end(nodes));
        dog_status->setScanData(current_scan_data, count);
        
        int t=0;
        if ( is_ready == false ) {
            prev_scan_data.clear();
            for (int i=0; i<current_scan_data.size(); i++) {
                float theta = current_scan_data[i].angle_z_q14 * 90.f / 16384.f; 
                float object_distance = current_scan_data[i].dist_mm_q2 / 4.0f;

                if (object_distance > 0) {
                    float x=0;
                    float y=0;
                    cv::Point2f point;
                    x = object_distance * cos(theta * (CV_PI/180)) + 500;
                    y = object_distance * sin(theta * (CV_PI/180)) + 500;
                    point.x = x * cos(rotate * (CV_PI/180)) - y * sin(rotate * (CV_PI/180));
                    point.y = x * sin(rotate * (CV_PI/180)) + y * cos(rotate * (CV_PI/180));
                    prev_scan_data.push_back( point );
                }
            }
            is_ready = true;
            continue;
        } 

        else {
            // for odometry
            pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan_point_cloud (new pcl::PointCloud<pcl::PointXYZ>(current_scan_data.size(),1)); 
            pcl::PointCloud<pcl::PointXYZ>::Ptr prev_scan_point_cloud (new pcl::PointCloud<pcl::PointXYZ>(prev_scan_data.size(),1));

            curr_scan_data.clear();
            for (int i=0; i<current_scan_data.size(); i++) {
                float theta = current_scan_data[i].angle_z_q14 * 90.f / 16384.f; 
                float object_distance = current_scan_data[i].dist_mm_q2 / 4.0f;
                if (object_distance > 0) {
                    cv::Point2f point;
                    float x = object_distance * cos(theta * (CV_PI/180)) + 500;
                    float y = object_distance * sin(theta * (CV_PI/180)) + 500;
                    point.x = x * cos(rotate * (CV_PI/180)) - y * sin(rotate * (CV_PI/180));
                    point.y = x * sin(rotate * (CV_PI/180)) + y * cos(rotate * (CV_PI/180));
                    curr_scan_data.push_back( point );
                }
            }

            int i = 0;

            for (auto& point : *current_scan_point_cloud) {
                point.x = curr_scan_data[i].x;
                point.y = curr_scan_data[i].y;
                point.z = 0.0f; // for 2D 
                i++;
            }
            
            i = 0;
            for (auto& point : *prev_scan_point_cloud) {
                point.x = prev_scan_data[i].x;
                point.y = prev_scan_data[i].y;
                point.z = 0.0f; // for 2D 
                i++;
            }

            // icp algorithm
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setMaximumIterations( 11 );
            icp.setInputSource(prev_scan_point_cloud);
            icp.setInputTarget(current_scan_point_cloud);
            pcl::PointCloud<pcl::PointXYZ> final;
            icp.align(final);
            icp.setMaximumIterations (1);
            Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
            
            // for debug 
            // std::cout << "transformation_matrix = " << transformation_matrix << std::endl; 
            current_location.x = current_location.x * transformation_matrix(0, 0) + current_location.y * transformation_matrix(0, 1) + transformation_matrix(0, 3);
            current_location.y = current_location.x * transformation_matrix(1, 0) + current_location.y * transformation_matrix(1, 1) + transformation_matrix(1, 3);

            dog_status->setCurrentLocation(current_location);

            prev_scan_data.clear();
            for (int i=0; i<curr_scan_data.size(); i++) {
                prev_scan_data.push_back( curr_scan_data[i] );
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void System::controlThread(std::shared_ptr<Control> control, std::shared_ptr<DogStatus> dog_status) {
    char command;
    bool status;
    bool is_in_dangerzone = false;
    bool is_safe_left = true;
    bool is_safe_right = true;
    
    const float object_collision_distance_threshold = 350;

    while (1) {
        if (!dog_status->getSystemStatus()) {
            printf("[SYSTEM] Exit Control Thread\n");
            break;
        }

        std::vector<sl_lidar_response_measurement_node_hq_t> current_scan_data = dog_status->getScanData();
        // our LIDAR 
        //
        //         0
        //        ****   
        // 270   ******    90
        //        ****   
        //         \/    
        //        180 
        //  ----------------
        // |                |
        // |    OUR ZONE    |
        // |                |
        //  ----------------

        // check 
        float max_left_object_distance = 0;
        float max_right_object_distance = 0;
        
        for (int i=0; i<current_scan_data.size(); i++) {
            float theta = current_scan_data[i].angle_z_q14 * 90.f / 16384.f; 
            // because, our range is 90 ~ 270
            if ( theta < 90) continue;
            if ( theta > 270) break;

            // object distance 
            float object_distance = current_scan_data[i].dist_mm_q2 / 4.0f;

            if (theta < 135) { // check colision warning
                if (object_distance != 0 && object_distance < object_collision_distance_threshold) {
                    is_safe_left = false;
                    if (max_left_object_distance < object_distance) {
                        max_left_object_distance = object_distance;
                    }
                }
            }
            else if (theta < 180) { // check colision warning
                if (object_distance != 0 && object_distance < object_collision_distance_threshold) {
                    is_in_dangerzone = true;
                }
            }
            else if (theta < 225) { // check colision warning
                if (object_distance != 0 && object_distance < object_collision_distance_threshold) {
                    is_in_dangerzone = true;
                }
            }
            else if (theta < 270) { // check colision warning
                if (object_distance != 0 && object_distance < object_collision_distance_threshold) {
                    is_safe_right = false;
                    if (max_right_object_distance < object_distance) {
                        max_right_object_distance = object_distance;
                    }
                }
            }
        }

        if ( is_in_dangerzone ) {
            if ( is_safe_right == true && is_safe_left == false ) { // safe right 
                command = TURN_RIGHT;
            }
            else if ( is_safe_left == true && is_safe_right == false ) { // safe left 
                command = TURN_LEFT;
            }
            else if ( is_safe_right == false && is_safe_left == false ) {
                // if dog can't turn left and turn right, dog need to turn arround
                // So Continue giving 's' to command, due to turn arround.
                command = TURN_RIGHT;
            }
            else if ( is_safe_right == true && is_safe_left == true ) {
                // if both is_safe_right and is_safe_left is true, we choice according to object distance .
                if (max_right_object_distance > max_left_object_distance) {
                    command = TURN_RIGHT;
                }
                else {
                    command = TURN_LEFT;
                }
            }
        } 
        else {
            command = GO_FORWARD;
        }

        // for Debug 
        // printf("[Debug] Is in Dangerzone : %s\n", is_in_dangerzone ? "Yes!" : "No!");
        // printf("[Debug] Safe Left : %s\n", is_safe_left ? "Yes!" : "No!");
        // printf("[Debug] Safe Right : %s\n", is_safe_right ? "Yes!" : "No!");
        // printf("[Debug] Command : %c\n", command);
        // printf("============================================================================\n");

        // init flag 
        is_in_dangerzone = false;
        is_safe_left = true;
        is_safe_right = true;

        max_left_object_distance = 0;
        max_right_object_distance = 0;

        status = control->sendToCommand(command);     

        if (!status) {
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    }
}
