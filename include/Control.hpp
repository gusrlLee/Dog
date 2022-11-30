#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include <unistd.h>       // Used for UART
#include <sys/fcntl.h>    // Used for UART
#include <termios.h>      // Used for UART
#include <string>

#include <iostream>
#include <thread>
#include <mutex>
#include <algorithm>
#include <vector>
#include <string>

#include "config/config.hpp"

class Control {
    public:
        Control(const char* port_path, speed_t baud_rate);
        ~Control() { close(m_fid); };
        // TRANSMISSION
        bool sendToCommand(char command);

    private:
        int m_fid;
        struct termios m_port_options;
};


#endif