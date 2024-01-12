cmake_minimum_required(VERSION 3.8)
project(vl53l5cx_mpu6050_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(VL_PLATFORM_PATH "${CMAKE_CURRENT_SOURCE_DIR}/en.STSW-IMG023/VL53L5CX_ULD_driver_1.3.10/Platform")
set(VL_API_PATH "${CMAKE_CURRENT_SOURCE_DIR}/en.STSW-IMG023/VL53L5CX_ULD_driver_1.3.10/VL53L5CX_ULD_API")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

file(GLOB_RECURSE VL53L5CX_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE VL53L5CX_PLATFORM_FILES ${VL_PLATFORM_PATH}/*.cpp)
file(GLOB_RECURSE VL53L5CX_API_FILES ${VL_API_PATH}/src/*.cpp)

set(SRC_FILE ${VL53L5CX_FILES} 
              ${VL53L5CX_API_FILES} ${VL53L5CX_PLATFORM_FILES})
  
add_executable(vl53l5cx_mpu6050_driver_node ${SRC_FILE})

ament_target_dependencies(
	vl53l5cx_mpu6050_driver_node
	rclcpp
	sensor_msgs
)

target_include_directories(vl53l5cx_mpu6050_driver_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${VL_API_PATH}/inc>
	$<BUILD_INTERFACE:${VL_PLATFORM_PATH}>
	$<INSTALL_INTERFACE:include>
)	

install(TARGETS vl53l5cx_mpu6050_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
