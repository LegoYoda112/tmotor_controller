cmake_minimum_required(VERSION 3.0.0)
project(tmotor_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
)

include_directories( include ${catkin_INCLUDE_DIRS} )

file(GLOB SOURCES "src/*.cpp")

# add_executable(testing ${SOURCES})

add_executable(motor_controller src/motor_controller.cpp src/MotorManager.cpp src/TMotor.cpp src/canbus.cpp)
# add_dependencies(motor_controller ${SOURCES})
target_link_libraries(motor_controller ${catkin_LIBRARIES})

add_executable(joystick_controller src/joystick_controller.cpp)
target_link_libraries(joystick_controller ${catkin_LIBRARIES})