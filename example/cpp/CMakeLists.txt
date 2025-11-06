cmake_minimum_required(VERSION 3.12)
project(airbot_hardware_cpp_example LANGUAGES CXX)

if(AIRBOT_HARDWARE_BUILD_STATIC)
  option(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC "Use static library" ON)
else()
  option(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC "Use static library" OFF)
endif()

# For Ubuntu 20.04
add_link_options(-pthread)

set(AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS -Wall -Werror -Wno-literal-suffix)

if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  if(NOT TARGET airbot_hardware_static)
    find_package(airbot_hardware CONFIG REQUIRED
                 COMPONENTS airbot_hardware_static)
    set(AIRBOT_HW_EXAMPLES_DEPS ${AIRBOT_HW_EXAMPLES_DEPS}
                                airbot_hardware::airbot_hardware_static pthread)
  else()
    set(AIRBOT_HW_EXAMPLES_DEPS ${AIRBOT_HW_EXAMPLES_DEPS}
                                airbot_hardware_static pthread)
  endif()
else()
  if(NOT TARGET airbot_hardware)
    find_package(airbot_hardware CONFIG REQUIRED COMPONENTS airbot_hardware)
    set(AIRBOT_HW_EXAMPLES_DEPS ${AIRBOT_HW_EXAMPLES_DEPS}
                                airbot_hardware::airbot_hardware pthread)
  else()
    set(AIRBOT_HW_EXAMPLES_DEPS ${AIRBOT_HW_EXAMPLES_DEPS} airbot_hardware
                                pthread)
  endif()
endif()

if(NOT TARGET argparse)
  find_package(argparse CONFIG REQUIRED)
  set(AIRBOT_HW_EXAMPLES_DEPS ${AIRBOT_HW_EXAMPLES_DEPS} argparse::argparse)
else()
  set(AIRBOT_HW_EXAMPLES_DEPS ${AIRBOT_HW_EXAMPLES_DEPS} argparse)
endif()

if(NOT TARGET Boost::asio)
  find_package(
    Boost
    COMPONENTS asio
    REQUIRED)
endif()
set(AIRBOT_HW_EXAMPLES_DEPS ${AIRBOT_HW_EXAMPLES_DEPS} Boost::asio)

if(NOT TARGET Boost::json)
  find_package(
    Boost
    COMPONENTS json
    REQUIRED)
endif()
set(AIRBOT_HW_EXAMPLES_DEPS ${AIRBOT_HW_EXAMPLES_DEPS} Boost::json)

add_executable(example_bipl src/bipl.cpp)
target_link_libraries(example_bipl PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_bipl
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_bipl PRIVATE -static)
endif()

add_executable(example_free_drive src/free_drive.cpp)
target_link_libraries(example_free_drive PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_free_drive
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_free_drive PRIVATE -static)
endif()

add_executable(example_free_drive_replay src/hardware/arm/free_drive_replay.cpp)
target_link_libraries(example_free_drive_replay
                      PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_free_drive_replay
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_free_drive_replay PRIVATE -static)
endif()

add_executable(example_comm_serial src/comm/serial.cpp)
target_link_libraries(example_comm_serial PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_comm_serial
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_comm_serial PRIVATE -static)
endif()

add_executable(example_comm_keyboard src/comm/keyboard.cpp)
target_link_libraries(example_comm_keyboard PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_comm_keyboard
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_comm_keyboard PRIVATE -static)
endif()

add_executable(example_comm_tty src/comm/tty.cpp)
target_link_libraries(example_comm_tty PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_comm_tty
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_comm_tty PRIVATE -static)
endif()

# add_executable(example_comm_modbus_rtu src/comm/modbus_rtu.cpp)
# target_link_libraries(example_comm_modbus_rtu PRIVATE
# ${AIRBOT_HW_EXAMPLES_DEPS}) target_compile_options(example_comm_modbus_rtu
# PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
# if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
# target_link_options(example_comm_modbus_rtu PRIVATE -static) endif()

add_executable(example_eef_E2 src/hardware/eef/E2.cpp)
target_link_libraries(example_eef_E2 PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_eef_E2
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_eef_E2 PRIVATE -static)
endif()

add_executable(example_eef_G2 src/hardware/eef/G2.cpp)
target_link_libraries(example_eef_G2 PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_eef_G2
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_eef_G2 PRIVATE -static)
endif()

add_executable(example_arm_return_zero src/hardware/arm/return_zero.cpp)
target_link_libraries(example_arm_return_zero
                      PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_arm_return_zero
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_arm_return_zero PRIVATE -static)
endif()

add_executable(example_arm_kbd_ctrl src/hardware/arm/kbd_ctrl.cpp)
target_link_libraries(example_arm_kbd_ctrl PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_arm_kbd_ctrl
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_arm_kbd_ctrl PRIVATE -static)
endif()

add_executable(example_arm_free_drive src/hardware/arm/free_drive.cpp)
target_link_libraries(example_arm_free_drive PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_arm_free_drive
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_arm_free_drive PRIVATE -static)
endif()

find_package(Boost COMPONENTS asio json)

add_executable(example_arm_swing_and_plot src/hardware/arm/swing_and_plot.cpp)
target_link_libraries(
  example_arm_swing_and_plot PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS} Boost::json
                                     Boost::asio)
target_compile_options(
  example_arm_swing_and_plot PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS}
                                     -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_arm_swing_and_plot PRIVATE -static)
endif()

add_executable(example_arm_free_drive_and_plot
               src/hardware/arm/free_drive_and_plot.cpp)
target_link_libraries(
  example_arm_free_drive_and_plot PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                          Boost::json Boost::asio)
target_compile_options(
  example_arm_free_drive_and_plot
  PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS} -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_arm_free_drive_and_plot PRIVATE -static)
endif()

# add_executable(example_play_button src/hardware/gpio/play_button.cpp)
# target_link_libraries(example_play_button PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
# target_compile_options(example_play_button PRIVATE
# ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
# if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC) target_link_options(example_play_button
# PRIVATE -static) endif()

# add_executable(example_play_led src/hardware/gpio/play_led_bipl1.cpp)
# target_link_libraries(example_play_led PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
# target_compile_options(example_play_led PRIVATE
# ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
# if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC) target_link_options(example_play_led
# PRIVATE -static) endif()

if(AIRBOT_HARDWARE_BUILD_SLAMTEC)
  add_executable(example_athena_revolve src/athena/revolve.cpp)
  target_link_libraries(example_athena_revolve
                        PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
  target_compile_options(example_athena_revolve
                         PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
  if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
    target_link_options(example_athena_revolve PRIVATE -static)
  endif()
endif()

add_executable(airbot_ins_hand_ctrl src/product/ins_hand_ctrl.cpp)
target_link_libraries(airbot_ins_hand_ctrl PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                   Boost::json Boost::asio)
target_compile_options(airbot_ins_hand_ctrl
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_ins_hand_ctrl PRIVATE -static)
endif()

add_executable(airbot_play_kbd_ctrl_1 src/product/play_kbd_ctrl_1.cpp)
target_link_libraries(airbot_play_kbd_ctrl_1 PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                     Boost::json Boost::asio)
target_compile_options(
  airbot_play_kbd_ctrl_1 PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS}
                                 -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_play_kbd_ctrl_1 PRIVATE -static)
endif()

add_executable(airbot_e2 src/product/e2.cpp)
target_link_libraries(airbot_e2 PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS} Boost::json
                                        Boost::asio)
target_compile_options(
  airbot_e2 PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS} -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_e2 PRIVATE -static)
endif()

add_executable(airbot_play_return_zero src/product/play_return_zero.cpp)
target_link_libraries(airbot_play_return_zero PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                      Boost::json Boost::asio)
target_compile_options(
  airbot_play_return_zero PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS}
                                  -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_play_return_zero PRIVATE -static)
endif()

add_executable(airbot_play_pos_ctrl src/product/play_pos_ctrl.cpp)
target_link_libraries(airbot_play_pos_ctrl PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                   Boost::json Boost::asio)
target_compile_options(
  airbot_play_pos_ctrl PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS}
                               -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_play_pos_ctrl PRIVATE -static)
endif()

add_executable(airbot_play_pos_show src/product/play_pos_show.cpp)
target_link_libraries(airbot_play_pos_show PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                   Boost::json Boost::asio)
target_compile_options(
  airbot_play_pos_show PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS}
                               -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_play_pos_show PRIVATE -static)
endif()

add_executable(example_1f1 src/hardware/arm/1f1.cpp)
target_link_libraries(example_1f1 PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                          Boost::json Boost::asio)
target_compile_options(
  example_1f1 PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS} -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_1f1 PRIVATE -static)
endif()

add_executable(airbot_g2_pos_ctrl src/product/g2_pos_ctrl.cpp)
target_link_libraries(airbot_g2_pos_ctrl PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                 Boost::json Boost::asio)
target_compile_options(
  airbot_g2_pos_ctrl PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS}
                             -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_g2_pos_ctrl PRIVATE -static)
endif()

add_executable(airbot_g2_pos_swing src/product/g2_pos_swing.cpp)
target_link_libraries(airbot_g2_pos_swing PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                  Boost::json Boost::asio)
target_compile_options(
  airbot_g2_pos_swing PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS}
                              -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_g2_pos_swing PRIVATE -static)
endif()

add_executable(airbot_play_mit_free_drive src/product/play_mit_free_drive.cpp)
target_link_libraries(
  airbot_play_mit_free_drive PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS} Boost::json
                                     Boost::asio)
target_compile_options(
  airbot_play_mit_free_drive PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS}
                                     -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_play_mit_free_drive PRIVATE -static)
endif()

add_executable(airbot_g2_force src/product/g2_force.cpp)
target_link_libraries(airbot_g2_force PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                              Boost::json Boost::asio)
target_compile_options(
  airbot_g2_force PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS}
                          -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_g2_force PRIVATE -static)
endif()

add_executable(airbot_play_mit_swing src/product/play_mit_swing.cpp)
target_link_libraries(airbot_play_mit_swing PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                    Boost::json Boost::asio)
target_compile_options(
  airbot_play_mit_swing PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS}
                                -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_play_mit_swing PRIVATE -static)
endif()

add_executable(airbot_1f1 src/product/1f1.cpp)
target_link_libraries(airbot_1f1 PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS} Boost::json
                                         Boost::asio)
target_compile_options(
  airbot_1f1 PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS} -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_1f1 PRIVATE -static)
endif()

add_executable(airbot_replay src/product/replay.cpp)
target_link_libraries(airbot_replay PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                            Boost::json Boost::asio)
target_compile_options(
  airbot_replay PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS}
                        -Wno-narrowing)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_replay PRIVATE -static)
endif()

add_executable(airbot_ins_hand_pose src/product/ins_hand_pose.cpp)
target_link_libraries(airbot_ins_hand_pose PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(airbot_ins_hand_pose
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_ins_hand_pose PRIVATE -static)
endif()

add_executable(airbot_ins_hand_greeting src/product/ins_hand_greeting.cpp)
target_link_libraries(
  airbot_ins_hand_greeting PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS} Boost::json
                                   Boost::asio)
target_compile_options(airbot_ins_hand_greeting
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_ins_hand_greeting PRIVATE -static)
endif()

add_executable(example_argparse src/argparse.cpp)
target_link_libraries(example_argparse PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS})
target_compile_options(example_argparse
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(example_argparse PRIVATE -static)
endif()

add_executable(airbot_brainco_hand_ctrl src/product/brain_co_hand_ctrl.cpp)
target_link_libraries(
  airbot_brainco_hand_ctrl PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS} Boost::json
                                   Boost::asio)
target_compile_options(airbot_brainco_hand_ctrl
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
target_compile_options(airbot_brainco_hand_ctrl PRIVATE $<$<CONFIG:Release>:-g>)
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_brainco_hand_ctrl PRIVATE -static)
endif()

add_executable(airbot_brainco_hand_pose src/product/brain_co_hand_pose.cpp)
target_link_libraries(
  airbot_brainco_hand_pose PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS} Boost::json
                                   Boost::asio)
target_compile_options(airbot_brainco_hand_pose
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_brainco_hand_pose PRIVATE -static)
endif()

add_executable(airbot_brainco_hand_greeting
               src/product/brain_co_hand_greeting.cpp)
target_link_libraries(
  airbot_brainco_hand_greeting PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS} Boost::json
                                       Boost::asio)
target_compile_options(airbot_brainco_hand_greeting
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_brainco_hand_greeting PRIVATE -static)
endif()

add_executable(airbot_ro_hand_ctrl src/product/ro_hand_ctrl.cpp)
target_link_libraries(airbot_ro_hand_ctrl PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                  Boost::json Boost::asio)
target_compile_options(airbot_ro_hand_ctrl
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_ro_hand_ctrl PRIVATE -static)
endif()

add_executable(airbot_ro_hand_pose src/product/ro_hand_pose.cpp)
target_link_libraries(airbot_ro_hand_pose PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                  Boost::json Boost::asio)
target_compile_options(airbot_ro_hand_pose
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_ro_hand_pose PRIVATE -static)
endif()

add_executable(airbot_rohand_arm_debug
               src/hardware/hand/airbot_rohand_arm_debug.cpp)
target_link_libraries(airbot_rohand_arm_debug PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                      Boost::json Boost::asio)
target_compile_options(airbot_rohand_arm_debug
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_rohand_arm_debug PRIVATE -static)
endif()

add_executable(airbot_ro_hand_greeting src/product/ro_hand_greeting.cpp)
target_link_libraries(airbot_ro_hand_greeting PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                      Boost::json Boost::asio)
target_compile_options(airbot_ro_hand_greeting
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_ro_hand_greeting PRIVATE -static)
endif()

add_executable(airbot_arm_get_params src/product/get_params.cpp)
target_link_libraries(airbot_arm_get_params PRIVATE ${AIRBOT_HW_EXAMPLES_DEPS}
                                                    Boost::json Boost::asio)
target_compile_options(airbot_arm_get_params
                       PRIVATE ${AIRBOT_HARDWARE_EXAMPLE_COMPILE_OPTIONS})
if(AIRBOT_HARDWARE_EXAMPLE_USE_STATIC)
  target_link_options(airbot_arm_get_params PRIVATE -static)
endif()

if(AIRBOT_HARDWARE_INSTALL)
  install(TARGETS example_free_drive RUNTIME DESTINATION bin)
  install(TARGETS example_comm_serial RUNTIME DESTINATION bin)
  install(TARGETS example_comm_keyboard RUNTIME DESTINATION bin)
  install(TARGETS example_comm_tty RUNTIME DESTINATION bin)
  install(TARGETS example_eef_E2 RUNTIME DESTINATION bin)
  install(TARGETS example_eef_G2 RUNTIME DESTINATION bin)
  install(TARGETS example_arm_return_zero RUNTIME DESTINATION bin)
  install(TARGETS example_arm_kbd_ctrl RUNTIME DESTINATION bin)
  install(TARGETS example_arm_swing_and_plot RUNTIME DESTINATION bin)
  install(TARGETS example_arm_free_drive_and_plot RUNTIME DESTINATION bin)
  # install(TARGETS example_play_button RUNTIME DESTINATION bin) install(TARGETS
  # example_play_led RUNTIME DESTINATION bin)
  if(AIRBOT_HARDWARE_BUILD_SLAMTEC)
    install(TARGETS example_athena_revolve RUNTIME DESTINATION bin)
  endif()
  install(TARGETS airbot_play_pos_ctrl RUNTIME DESTINATION bin)
  install(TARGETS airbot_play_pos_show RUNTIME DESTINATION bin)
  install(TARGETS airbot_play_return_zero RUNTIME DESTINATION bin)
  install(TARGETS example_1f1 RUNTIME DESTINATION bin)
  install(TARGETS example_argparse RUNTIME DESTINATION bin)
  install(TARGETS airbot_play_kbd_ctrl_1 RUNTIME DESTINATION bin)
  install(TARGETS airbot_e2 RUNTIME DESTINATION bin)
  install(TARGETS airbot_g2_pos_ctrl RUNTIME DESTINATION bin)
  install(TARGETS airbot_g2_pos_swing RUNTIME DESTINATION bin)
  install(TARGETS airbot_play_mit_free_drive RUNTIME DESTINATION bin)
  install(TARGETS airbot_g2_force RUNTIME DESTINATION bin)
  install(TARGETS airbot_play_mit_swing RUNTIME DESTINATION bin)
  install(TARGETS airbot_1f1 RUNTIME DESTINATION bin)
  install(TARGETS airbot_replay RUNTIME DESTINATION bin)
  install(TARGETS airbot_ins_hand_ctrl RUNTIME DESTINATION bin)
  install(TARGETS airbot_ins_hand_pose RUNTIME DESTINATION bin)
  install(TARGETS airbot_ins_hand_greeting RUNTIME DESTINATION bin)
  install(TARGETS airbot_brainco_hand_ctrl RUNTIME DESTINATION bin)
  install(TARGETS airbot_brainco_hand_pose RUNTIME DESTINATION bin)
  install(TARGETS airbot_brainco_hand_greeting RUNTIME DESTINATION bin)
  install(TARGETS airbot_ro_hand_ctrl RUNTIME DESTINATION bin)
  install(TARGETS airbot_ro_hand_pose RUNTIME DESTINATION bin)
  install(TARGETS airbot_ro_hand_greeting RUNTIME DESTINATION bin)
  install(TARGETS airbot_arm_get_params RUNTIME DESTINATION bin)
  install(TARGETS airbot_rohand_arm_debug RUNTIME DESTINATION bin)
  if(AIRBOT_HARDWARE_INSTALL_DEV)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
            DESTINATION share/airbot_hardware/examples/cpp)
  endif()
endif()
