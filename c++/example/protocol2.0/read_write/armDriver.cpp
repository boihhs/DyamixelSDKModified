#include "armDriver.h"
#include <iostream>

ArmInterface::ArmInterface() {    
    setup();
}

bool ArmInterface::setup() {
    // Setup motors
    for (size_t i = 0; i < motors.size(); ++i) {
        motors[i].DXL_ID = i + 1;
        motors[i].dxl_present_position = 0;
    }

    /// Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = dynamixel::PortHandler::getPortHandler(port_name);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Initialize GroupSyncWrite instance
    groupSyncWrite = new dynamixel::GroupSyncWrite(
            portHandler, packetHandler,
            ADDR_GOAL_POSITION, 4);

    // Initialize Groupsyncread instance for Present Position
    groupSyncRead = new dynamixel::GroupSyncRead(
            portHandler, packetHandler,
            ADDR_PRESENT_POSITION, 4);        
    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Enable Dynamixel Torque
    for(size_t i = 0; i < motors.size(); ++i){
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motors[i].DXL_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
            packetHandler->getRxPacketError(dxl_error);
        }
        else
        {
            printf("Dynamixel#%d has been successfully connected \n", motors[i].DXL_ID);
        }
        
        dxl_addparam_result = groupSyncRead->addParam(motors[i].DXL_ID);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", motors[i].DXL_ID);
            return 0;
        }
    }

    return 1;
}

bool ArmInterface::send(const std::vector<int32_t>& goals) {

    for(size_t i = 0; i < motors.size(); ++i){
        // Allocate goal position value into byte array
        uint8_t param_goal_position[4];
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goals[i]));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goals[i]));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goals[i]));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goals[i]));

        // Add Dynamixel goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite->addParam(motors[i].DXL_ID, param_goal_position);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motors[i].DXL_ID);
            return 0;
        }
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

    // Clear syncwrite parameter storage
    groupSyncWrite->clearParam();

    return 1;


    
}

bool ArmInterface::recv() {

    // Syncread present position
    dxl_comm_result = groupSyncRead->txRxPacket();
    
    if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

    for(size_t i = 0; i < motors.size(); ++i){
        // Check if groupsyncread data of Dynamixel is available
        dxl_getdata_result = groupSyncRead->isAvailable(motors[i].DXL_ID, ADDR_PRESENT_POSITION, 4);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", motors[i].DXL_ID);
            return 0;
        }
        motors[i].dxl_present_position = groupSyncRead->getData(motors[i].DXL_ID, ADDR_PRESENT_POSITION, 4);

    }
    return 1;
    
}

bool ArmInterface::exit(int32_t t) {
    if (t > 0){

        control(exitPos, t);

    }
    

    for(size_t i = 0; i < motors.size(); ++i){
        // Disable Dynamixel#1 Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motors[i].DXL_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
            packetHandler->getRxPacketError(dxl_error);
        }
    }
    // Close port
    portHandler->closePort();

    return 1;
    
}

bool ArmInterface::control(const std::vector<int32_t>& goals, int32_t t){

    if (t > 0){

        auto start_time = std::chrono::steady_clock::now();
        std::vector<int32_t> inBet(7, 0);
        std::vector<int32_t> oldBet(7, 0);

        for(size_t i = 0; i < motors.size(); ++i){
            oldBet[i] = motors[i].dxl_present_position;
        }

        do{
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = now - start_time;
            double time = elapsed_seconds.count();
            if (time > t){
                time = t;
            }
            for(size_t i = 0; i < motors.size(); ++i){
                inBet[i] = (time/t)*(goals[i]) + (1 - time/t)*(oldBet[i]);
            }

            send(inBet);
            
            recv();
        }while(getError(goals));

    }else{
        do{
          send(goals);
          recv();
        }while(getError(goals));
    }

}


bool ArmInterface::getError(const std::vector<int32_t>& goals){
    double error = 0;
    for(size_t i = 0; i < motors.size(); ++i){
        error = motors[i].dxl_present_position - goals[i];
        if (error > DXL_MOVING_STATUS_THRESHOLD){
            return 1;
        }
        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", motors[i].DXL_ID, goals[i], motors[i].dxl_present_position);
    }
    return 0;
}
