if (TARGET tinyusb_device)

add_custom_target(pre_build_version_update)
add_custom_command(
    TARGET pre_build_version_update
    PRE_BUILD
    COMMAND ${CMAKE_COMMAND}
        -DGEN_DIR:PATH="${CMAKE_CURRENT_LIST_DIR}/generated"
        -P "${ESE5190_FINALPROJ_DIR}/auto_version.cmake"
)

set(PARAM_ASSERTIONS_ENABLE_ALL 1)
add_executable(robot_arm)
add_dependencies(robot_arm pre_build_version_update)
target_sources(robot_arm PRIVATE 
    robot_arm.c
    devices/PCA9685.c
    devices/arducam.c
    devices/joystick.c
    motion_control/ik_calculations.c
    motion_control/joint_control.c
)

#example PIO header generation
#pico_generate_pio_header(robot_arm ${CMAKE_CURRENT_LIST_DIR}/devices/camera.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated) # PICO SDK (PIO)

target_include_directories(robot_arm PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/devices/include/.
    ${CMAKE_CURRENT_LIST_DIR}/motion_control/include/.
    ${CMAKE_CURRENT_LIST_DIR}/generated/.
)
target_link_libraries(robot_arm PRIVATE 
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_i2c
    hardware_adc
)

pico_add_extra_outputs(robot_arm)

pico_enable_stdio_usb(robot_arm 1)
pico_enable_stdio_uart(robot_arm 0)



elseif(PICO_ON_DEVICE)
    message(WARNING "not building robot_arm
 because TinyUSB submodule is not initialized in the SDK")
endif()