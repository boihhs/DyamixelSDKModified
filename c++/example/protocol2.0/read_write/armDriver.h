#ifndef ARMDRIVER_H
#define ARMDRIVER_H

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <array>  // for std::array

#include "dynamixel_sdk.h"  // Uses DYNAMIXEL SDK library

// Define a simple struct for a motor.
struct armMotor {
    int dxl_goal_position[2];
    uint8_t DXL_ID;
    int32_t dxl_present_position;
};

class ArmInterface {
public:
    ArmInterface();
    ~ArmInterface() {}

    void setup();
    void recv();
    void send(int32_t goal);
    void exit();

    std::array<armMotor, 7> motors;

private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    uint16_t ADDR_TORQUE_ENABLE = 64;
    uint16_t ADDR_GOAL_POSITION = 116;
    uint16_t ADDR_PRESENT_POSITION = 132;
    int MINIMUM_POSITION_LIMIT = 0;    // Refer to the Minimum Position Limit of product eManual.
    int MAXIMUM_POSITION_LIMIT = 4095; // Refer to the Maximum Position Limit of product eManual.
    int BAUDRATE = 57600;
    int dxl_comm_result = -1001;
    float protocol_version = 2.0;
    int32_t dxl_present_position = 0;
    uint8_t dxl_error = 0;
    char *port_name = (char*)"/dev/ttyUSB0";
};

#endif // ARMDRIVER_H
