cmake_minimum_required(VERSION 3.16)
project(airbot_hardware_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(airbot_hardware CONFIG REQUIRED)
find_package(Boost REQUIRED)

add_compile_options(-Wall -Werror -Wno-literal-suffix)
add_link_options(-pthread)

macro(configure_executable target src_file)
  add_executable(${target} ${src_file})

  ament_target_dependencies(${target} rclcpp sensor_msgs)

  target_link_libraries(${target} airbot_hardware::airbot_hardware
                        Boost::headers pthread)

  if(AIRBOT_HARDWARE_BUILD_STATIC)
    target_link_libraries(${target} airbot_hardware::airbot_hardware_static)
    target_compile_definitions(${target} PRIVATE AIRBOT_HARDWARE_BUILD_STATIC)
  endif()

  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endmacro()

configure_executable(airbot_play_pos_ctrl src/play_pos_ctrl.cpp)
configure_executable(airbot_play_pos_ctrl_with_g2 src/play_pos_ctrl_with_g2.cpp)
configure_executable(airbot_play_pos_ctrl_with_e2 src/play_pos_ctrl_with_e2.cpp)
configure_executable(airbot_eef_g2 src/G2.cpp)
configure_executable(airbot_eef_e2 src/E2.cpp)
configure_executable(airbot_replay src/replay.cpp)
configure_executable(airbot_play_free_drive src/play_free_drive.cpp)
configure_executable(airbot_play_pos_ctrl_mit src/play_pos_ctrl_mit.cpp)

ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(airbot_hardware)
ament_export_dependencies(Boost)

ament_package()
