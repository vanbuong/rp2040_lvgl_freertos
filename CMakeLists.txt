cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# Pull in FreeRTOS
include(FreeRTOS_Kernel_import.cmake)

project(example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(main_blinky
        main.c
        main_blinky.c
        )

target_compile_definitions(main_blinky PRIVATE
        mainCREATE_SIMPLE_BLINKY_DEMO_ONLY=1
        )

target_include_directories(main_blinky PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../../../../Common/include)

target_link_libraries(main_blinky pico_stdlib FreeRTOS-Kernel FreeRTOS-Kernel-Heap1)
pico_add_extra_outputs(main_blinky)