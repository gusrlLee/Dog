#include "Control.hpp"

Control::Control(const char* port_path, speed_t baud_rate) {
    int ii, jj, kk;

    // for init check 
    m_fid = -1;
    tcgetattr(m_fid, &m_port_options);
    m_fid = open(port_path, O_RDWR | O_NOCTTY );

    tcflush(m_fid, TCIFLUSH);
 	tcflush(m_fid, TCIOFLUSH);
    usleep(1000000);
    
    // this here 
    if (m_fid == -1) {
		printf("\n[ERROR]: Unable to open UART.  Ensure it is not in use by another application\n");
        exit(2);
	}

    m_port_options.c_cflag &= ~PARENB;                         // Disables the Parity Enable bit(PARENB),So No Parity
    m_port_options.c_cflag &= ~CSTOPB;                         // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
    m_port_options.c_cflag &= ~CSIZE;                          // Clears the mask for setting the data size
    m_port_options.c_cflag |= CS8;                             // Set the data bits = 8
    m_port_options.c_cflag &= ~CRTSCTS;                        // No Hardware flow Control
    m_port_options.c_cflag |= CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines
    m_port_options.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable XON/XOFF flow control both input & output
    m_port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non Cannonical mode
    m_port_options.c_oflag &= ~OPOST;

    m_port_options.c_lflag =  0;

    m_port_options.c_cc[VMIN] = VMINX;       
    m_port_options.c_cc[VTIME] = 0;

    cfsetispeed(&m_port_options, baud_rate);    // Set Read  Speed 
    cfsetospeed(&m_port_options, baud_rate);

    int att = tcsetattr(m_fid, TCSANOW, &m_port_options);
    if (att != 0 ) {
        printf("\n[ERROR]: in Setting port attributes\n");
    }

    // Flush Buffers
    tcflush(m_fid, TCIFLUSH);
    tcflush(m_fid, TCIOFLUSH);

    // 0.5 sec delay
    usleep(500000); 

    bool init_flag = sendToCommand(START_FLAG);
    if (init_flag == false) {
        printf("\n[ERROR]: CANNOT init our dogs, Try Again!\n");
        exit(1);
    }
}

bool Control::sendToCommand(char command) {
    // this is not connected Arduino
    if (m_fid == -1) return false;
    
    unsigned char tx_buffer[2];
    unsigned char *p_tx_buffer;
    p_tx_buffer = &tx_buffer[0];
    
    *p_tx_buffer++ = command;

    int count = write(m_fid, &tx_buffer, (p_tx_buffer - & tx_buffer[0]));
    // UART TX error status 
    if (count < 0) return false; 

    return true;
}