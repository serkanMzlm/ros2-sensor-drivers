cmake_minimum_required(VERSION 3.8)
project(vl53l5cx_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(VL_PLATFORM_PATH "${CMAKE_CURRENT_SOURCE_DIR}/en.STSW-IMG023/VL53L5CX_ULD_driver_1.3.10/Platform")
set(VL_API_PATH "${CMAKE_CURRENT_SOURCE_DIR}/en.STSW-IMG023/VL53L5CX_ULD_driver_1.3.10/VL53L5CX_ULD_API")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

target_include_directories(vl53l5cx_lib PUBLIC 
                ${CMAKE_CURRENT_SOURCE_DIR}/inc
                ${VL_PLATFORM_PATH}
                ${VL_API_PATH}/inc
)

ament_package()
