cmake_minimum_required(VERSION 3.8)
project(pca9685_servos_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Servo.msg"
  "msg/ServoCalibration.msg"
  "msg/ServoByTicks.msg"
)

ament_package()