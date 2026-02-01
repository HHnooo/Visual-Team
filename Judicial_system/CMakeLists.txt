cmake_minimum_required (VERSION
3.10
)
project (RobotRefereeSystem)

set (CMAKE_CXX_STANDARD
17
)

add_executable(robot_referee_system
               main.cpp
               robot.cpp
               robot_manager.cpp)

target_include_directories(robot_referee_system PRIVATE $ { CMAKE_CURRENT_SOURCE_DIR }
)