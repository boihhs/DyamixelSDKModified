#include "armDriver.h"
#include <iostream>
#include <cstdio>


#include <termios.h>
#include <unistd.h>
int getch() {
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

int main() {
    int index = 0;
    // Create an instance of ArmInterface. The constructor will call setup().
    ArmInterface arm;

    std::cout << "Initialization complete.\n";
    std::cout << "Press any key to send goal position command (Press ESC to exit)...\n";

    while (true) {
        int key = getch();
        // ESC key (ASCII value 27) terminates the loop.
        if (key == 27) {
            break;
        }
        
        // Send the pre-configured goal position to each motor.
        arm.send(arm.motors[0].dxl_goal_position[index]);
        
        // Read and display the present positions from each motor.
        arm.recv();
        
        std::cout << "\nPress any key to send command again or ESC to exit...\n";
        if(index == 0){
          index = 1;
        }else{
          index = 0;
        }
    }
    
    // Disable torque on each motor and close the communication port.
    arm.exit();

    std::cout << "Program terminated. Press any key to exit.\n";
    getch();
    return 0;
}