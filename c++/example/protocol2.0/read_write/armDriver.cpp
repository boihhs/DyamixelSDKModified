#include "armDriver.h"
#include <iostream>

ArmInterface::ArmInterface() {    
    setup();
}

void ArmInterface::setup() {
    // Initialize each motor's settings.
    for (size_t i = 0; i < motors.size(); ++i) {
        motors[i].DXL_ID = i + 1;
        motors[i].dxl_goal_position[0] = MINIMUM_POSITION_LIMIT;
        motors[i].dxl_goal_position[1] = MAXIMUM_POSITION_LIMIT;
    }

    // Obtain the port handler and packet handler without dereferencing.
    portHandler = dynamixel::PortHandler::getPortHandler(port_name);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);

    // Try opening the port.
    if (portHandler->openPort()) {
        printf("Succeeded to open port %s!\n", port_name);
    } else {
        printf("Failed to open the port!\n");
        return;
    }
    
    // Set the baudrate.
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    } else {
        printf("Failed to change the baudrate!\n");
        return;
    }

    // Enable torque for each motor.
    for (size_t i = 0; i < motors.size(); ++i) {
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, motors[i].DXL_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("Failed to enable torque for motor %d: %s\n", 
                   motors[i].DXL_ID, packetHandler->getTxRxResult(dxl_comm_result));
        } else {
            printf("Torque enabled for motor %d\n", motors[i].DXL_ID);
        }
    }
}

void ArmInterface::send(int32_t goal) {
    // For a simple example, send the first goal position (e.g., MINIMUM_POSITION_LIMIT)
    for (size_t i = 0; i < motors.size(); ++i) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler, motors[i].DXL_ID, ADDR_GOAL_POSITION, goal, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("Failed to send goal position to motor %d: %s\n", 
                   motors[i].DXL_ID, packetHandler->getTxRxResult(dxl_comm_result));
        } else {
            printf("Goal position sent to motor %d: %d\n", motors[i].DXL_ID, goal);
        }
    }
}

void ArmInterface::recv() {
    // Read and print the present position from each motor.
    for (size_t i = 0; i < motors.size(); ++i) {
        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler, motors[i].DXL_ID, ADDR_PRESENT_POSITION, 
            (uint32_t*)&dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("Failed to read present position from motor %d: %s\n", 
                   motors[i].DXL_ID, packetHandler->getTxRxResult(dxl_comm_result));
        } else {
            printf("Motor %d present position: %d\n", motors[i].DXL_ID, dxl_present_position);
        }
    }
}

void ArmInterface::exit() {
    // Disable torque for each motor.
    for (size_t i = 0; i < motors.size(); ++i) {
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, motors[i].DXL_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("Failed to disable torque for motor %d: %s\n", 
                   motors[i].DXL_ID, packetHandler->getTxRxResult(dxl_comm_result));
        } else {
            printf("Torque disabled for motor %d\n", motors[i].DXL_ID);
        }
    }
    portHandler->closePort();
}
