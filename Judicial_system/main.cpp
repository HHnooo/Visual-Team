#include "robot_manager.h"
#include <iostream>

int main() {
    RobotManager manager;
    manager.process_input_commands();
    manager.output_destroyed_list();
    return 0;
}

