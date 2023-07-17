cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

set(FREERTOS_KERNEL_PATH "D:\\Project\\FreeRTOS\\FreeRTOS-SMP-Demos\\FreeRTOS\\Source")
# Pull in FreeRTOS
include(FreeRTOS_Kernel_import.cmake)
# include(LVGL_import.cmake)

project(lvgl_freertos C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


pico_sdk_init()

add_compile_options (-fdiagnostics-color=always)
add_definitions(-DLV_FONT_MONTSERRAT_8=1
-DLV_FONT_MONTSERRAT_10=1
-DLV_FONT_MONTSERRAT_26=1
-DPICO_STACK_SIZE=0x1000
-DPICO_HEAP_SIZE=0x4000
)

add_subdirectory(lvgl)

add_executable(lvgl_freertos
        main.c
        main_blinky.c
        st7789.c
        )

pico_set_program_name(lvgl_freertos "lvgl_freertos")
pico_set_program_version(lvgl_freertos "0.1")

pico_enable_stdio_uart(lvgl_freertos 0)
pico_enable_stdio_usb(lvgl_freertos 1)

target_compile_definitions(lvgl_freertos PRIVATE
        mainCREATE_SIMPLE_BLINKY_DEMO_ONLY=1
        )

target_include_directories(lvgl_freertos PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../../../../Common/include)

target_link_libraries(lvgl_freertos
        pico_stdlib
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap1
        lvgl
        lvgl_examples
        lvgl_demos
        hardware_spi
        hardware_watchdog
        hardware_rtc
        hardware_pwm
        pico_time)

pico_add_extra_outputs(lvgl_freertos)