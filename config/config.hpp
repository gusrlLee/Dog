#ifndef CONFIG_H
#define CONFIG_H

// for LiDAR 
#define LIDAR_BAUDRATE 115200
#define MAP_PIXEL_RATIO 0.05
#define LIDAR_PIXEL_RATIO 0.1
#define LIDAR_MAX_DISTANCE 1500
#define COLLISION_DISTANCE_THRESHOLD 250

#define _countof( _Array ) (int)(sizeof(_Array) / sizeof(_Array[0]))

// for Serial Communication 
#define UART_BAUDRATE B115200
#define VMINX 1 
#define NSERIAL_CHAR 256


// command 
#define START_FLAG 's'
#define UP_FLAG 'u'
#define GO_FORWARD 'f'
#define TURN_RIGHT 'r'
#define TURN_LEFT 'l'
#define END_FLAG 'e'

#endif