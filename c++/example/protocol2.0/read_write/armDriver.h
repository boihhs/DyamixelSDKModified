#ifndef ARMDRIVER_H
#define ARMDRIVER_H


#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>
#include <array>  // for std::array

#include "dynamixel_sdk.h"  // Uses DYNAMIXEL SDK library

#include <chrono>


// Define a simple struct for a motor.
struct armMotor {
    uint8_t DXL_ID;
    int32_t dxl_present_position;
};

class ArmInterface {
public:
    ArmInterface();
    ~ArmInterface() {
        delete groupSyncRead;
        delete groupSyncWrite;
    }

    bool setup();
    bool recv();
    bool send(const std::vector<int32_t>& goals);
    bool control(const std::vector<int32_t>& goals, int32_t t);
    bool exit(int32_t t);
    bool getError(const std::vector<int32_t>& goals);

    std::array<armMotor, 7> motors;

    int getch()
    {
        struct termios oldt, newt;
        int ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncWrite *groupSyncWrite;
    dynamixel::GroupSyncRead *groupSyncRead;
    uint16_t ADDR_TORQUE_ENABLE = 64;
    uint16_t ADDR_GOAL_POSITION = 116;
    uint16_t ADDR_PRESENT_POSITION = 132;
    int MINIMUM_POSITION_LIMIT = 0;    // Refer to the Minimum Position Limit of product eManual.
    int MAXIMUM_POSITION_LIMIT = 4095; // Refer to the Maximum Position Limit of product eManual.
    int BAUDRATE = 1000000;
    int DXL_MOVING_STATUS_THRESHOLD = 50;
    int dxl_comm_result = -1001;
    float PROTOCOL_VERSION = 2.0;
    int32_t dxl_present_position = 0;
    uint8_t dxl_error = 0;
    char *port_name = (char*)"/dev/ttyUSB0";
    bool dxl_addparam_result = false;  
    bool dxl_getdata_result = false;  

    std::vector<int32_t> exitPos = {2056, 802, 802, 3107, 3107, 2000, 2228};
    std::vector<int32_t> startPos = {2048, 2048, 2048, 2048, 2048, 2048, 2048};


};

#endif // ARMDRIVER_H
