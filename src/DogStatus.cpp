#include "DogStatus.hpp"

DogStatus::DogStatus() {
    is_working = false;
}

// =============================================================================
// for current frame 
cv::Mat DogStatus::getCurrentFrame() {
    std::lock_guard<std::mutex> lock(current_frame_mutex);
    return m_current_frame.clone();
}

void DogStatus::setCurrentFrame(cv::Mat current_frame) {    
    std::lock_guard<std::mutex> lock(current_frame_mutex);
    m_current_frame = current_frame;
}

// =============================================================================
// for start flag 
bool DogStatus::getSystemStatus() {
    std::lock_guard<std::mutex> lock(is_working_mutex);
    return is_working;
}

void DogStatus::setSystemStatus(bool status) {
    std::lock_guard<std::mutex> lock(is_working_mutex);
    is_working = status;
}


// =============================================================================
// for current location
cv::Point2f DogStatus::getCurrentLocation() {
    std::lock_guard<std::mutex> lock(current_location_mutex);
    return m_current_location;
}


void DogStatus::setCurrentLocation(cv::Point2f current_location) {
    std::lock_guard<std::mutex> lock(current_location_mutex);
    m_current_location = current_location;
}

// =============================================================================
// for lidar data 
std::vector<sl_lidar_response_measurement_node_hq_t> DogStatus::getScanData() {
    std::lock_guard<std::mutex> lock(current_scan_data_mutex);
    return m_current_scan_data; 
}

void DogStatus::setScanData(std::vector<sl_lidar_response_measurement_node_hq_t> current_lidar_scan_data, size_t count) {
    std::lock_guard<std::mutex> lock(current_scan_data_mutex);
    m_current_scan_data = current_lidar_scan_data;
}

// =============================================================================
