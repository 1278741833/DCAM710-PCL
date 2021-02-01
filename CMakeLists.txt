cmake_minimum_required(VERSION 3.5.1)

# project set
set(PROJ_NAME capture_pico_zense)
project(${PROJ_NAME})
add_executable(
    ${PROJ_NAME} 
    src/Vzense.cpp
    src/main.cpp
    src/util.cpp
    src/commander.cpp
    src/ini.c
    src/INIReader.cpp
    src/network.cpp
    src/crc32.cpp
    src/net.cpp
    )

include_directories(include)
include_directories(include/google_log)

FIND_LIBRARY(GLOG_LIB glog ./include/google_log NO_DEFAULT_PATH)
FIND_LIBRARY(GLOGR_LIB glogr ./include/google_log NO_DEFAULT_PATH)
#link_libraries(${GLOG_LIB} ${GLOGR_LIB})
set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -L/home/zxy/tmp/capture_pico_zense/Lib -lvzense_api -lImgPreProcess")
#
target_link_libraries(${PROJ_NAME} ${GLOG_LIB} ${GLOGR_LIB})

find_package(PkgConfig REQUIRED)

pkg_check_modules(OpenCV REQUIRED opencv)
include_directories(${OpenCV_CFLAGS})
target_link_libraries(${PROJ_NAME} ${OpenCV_LDFLAGS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJ_NAME} ${PCL_LIBRARIES})

pkg_check_modules(PicoZense vzense REQUIRED)
include_directories(${PicoZense_INCLUDE_DIRS} Lib)
target_link_libraries(${PROJ_NAME} ${PicoZense_LDFLAGS})
