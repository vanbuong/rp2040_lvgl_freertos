# This can be dropped into an external project to help locate the FreeRTOS kernel
# It should be include()ed prior to project(). Alternatively this file may
# or the CMakeLists.txt in this directory may be included or added via add_subdirectory
# respectively.

if (NOT LWESP_PATH)
    message(FATAL_ERROR "LWESP location was not specified. Please set LWESP_PATH.")
endif()

set(LWESP_PATH "${LWESP_PATH}" CACHE PATH "Path to the LWESP library")

get_filename_component(LWESP_PATH "${LWESP_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
if (NOT EXISTS ${LWESP_PATH})
    message(FATAL_ERROR "Directory '${LWESP_PATH}' not found")
endif()

set(LWESP_PATH ${LWESP_PATH} CACHE PATH "Path to the LWESP" FORCE)

add_subdirectory(${LWESP_PATH} LWESP)