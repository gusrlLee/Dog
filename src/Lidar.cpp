#include "Lidar.hpp"

using namespace rp::standalone::rplidar;

Lidar::Lidar(std::string lidar_path, int baud_rate) {
    m_drv = *createLidarDriver();
    bool connect_success = false;

    if (!m_drv) {
        fprintf(stderr, "[ERROR]: insufficent memory, exit\n");
        exit(-2);
    }

    m_channel = (*createSerialPortChannel(lidar_path, baud_rate));

    if (SL_IS_OK((m_drv)->connect(m_channel))) {
        m_op_result = m_drv->getDeviceInfo(m_devinfo);

        if (SL_IS_OK(m_op_result)) {
            connect_success = true;
        }
        else {
            delete m_drv;
            m_drv = NULL;
        }   
    }

    if(!connect_success){
        std::cout << "\n[ERROR]: connect status: " << connect_success << std::endl;
        std::cerr << "[ERROR]: CANNOT CONNECT PORT!!\n" << std::endl;
        exit(2);
    }
    if (checkSLAMTECLIDARHealth(m_drv)){
        m_drv->setMotorSpeed();   
        m_drv->startScan(0, 1);
    }
}

Lidar::~Lidar(){
    m_drv->stop();
    m_drv->setMotorSpeed(0);

    if(m_drv)
        delete m_drv;
    m_drv = NULL;
}

size_t Lidar::grabScanedLidarData(
    sl_lidar_response_measurement_node_hq_t* nodes,
    size_t counts
) {
    sl_lidar_response_measurement_node_hq_t nodes_buf[8192];
    size_t result_counts = _countof(nodes);
    m_op_result = m_drv->grabScanDataHq(nodes, counts);

    if (SL_IS_OK(m_op_result)) {
        m_drv->ascendScanData(nodes, counts);
    }

    size_t temp = counts;
    return temp;
}

bool Lidar::checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        // printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "[ERROR], slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "[ERROR], cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}