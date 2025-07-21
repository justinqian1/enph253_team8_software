cmake_minimum_required(VERSION 3.31)
project(robot_codebase)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)
include_directories(src/components)
include_directories(src/hardware)
include_directories(src/interfaces)

add_executable(robot_codebase
        src/components/Claw.cpp
        src/components/Claw.h
        src/components/DriveMotors.cpp
        src/components/DriveMotors.h
        src/hardware/CustomServo.cpp
        src/hardware/CustomServo.h
        src/hardware/Motor.cpp
        src/hardware/Motor.h
        src/interfaces/motor_interface.h
        src/interfaces/servo_interface.h
        src/main.cpp
        src/PinAssignments.h
        test/hbridge_test.cpp)
