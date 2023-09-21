add_library(mpu6050-lib INTERFACE)
target_include_directories(mpu6050-lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(mpu6050-lib INTERFACE
    hardware_i2c
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4)
target_sources(mpu6050-lib INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mpu6050.c
    ${CMAKE_CURRENT_LIST_DIR}/mpu6050.h)