# This can be dropped into an external project to help locate the FreeRTOS kernel
# It should be include()ed prior to project(). Alternatively this file may
# or the CMakeLists.txt in this directory may be included or added via add_subdirectory
# respectively.

if (NOT LVGL_PATH)
    message(FATAL_ERROR "LVGL location was not specified. Please set LVGL_PATH.")
endif()

set(LVGL_PATH "${LVGL_PATH}" CACHE PATH "Path to the LVGL library")

get_filename_component(LVGL_PATH "${LVGL_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
if (NOT EXISTS ${LVGL_PATH})
    message(FATAL_ERROR "Directory '${LVGL_PATH}' not found")
endif()

set(LVGL_PATH ${LVGL_PATH} CACHE PATH "Path to the LVGL" FORCE)

add_subdirectory(${LVGL_PATH} LVGL)