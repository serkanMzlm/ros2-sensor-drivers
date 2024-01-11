#include "vl53l5cx_driver_node.hpp"

#include <chrono>

VL53L5CXDriver::VL53L5CXDriver(): Node("VL53L5CX_node"){
    if(init() < 0){
        exit(-1);
    }
    range_pub = this->create_publisher<RangeMsg>("range", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(
                        &VL53L5CXDriver::getRange, this));
}

int VL53L5CXDriver::init(){
    if(vl53l5cx_comms_init(&dev.platform)){
        reportError(errno, "Device's initialization failed");
        return -1;
    }
    status = vl53l5cx_is_alive(&dev, &isAlive);
    if(!isAlive || status){
        reportError(errno);
        return -1;
    }
    status = vl53l5cx_init(&dev);
    usleep(DELAY);
    status = vl53l5cx_set_ranging_frequency_hz(&dev, 20);
    usleep(DELAY);
    vl53l5cx_set_resolution(&dev, VL53L5CX_RESOLUTION_4X4);
    usleep(DELAY);
    status = vl53l5cx_start_ranging(&dev);
    vl53l5cx_check_data_ready(&dev, &isReady);
    usleep(DELAY);
    return 0;
}

void VL53L5CXDriver::getRange(){
    RangeMsg msg;
    status = vl53l5cx_check_data_ready(&dev, &isReady);
    // int sum;
    if(isReady){ 
        vl53l5cx_get_ranging_data(&dev, &Results);
        for(int a = 0; a < 16; a++){
			if(a % 8 == 0) std::cout << "\n";
            std::cout <<"|" << std::setw(4) << 
                    Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*a] << " mm|";
		}
    }
}

void VL53L5CXDriver::reportError(int error, std::string error_info ){
	std::cerr << "Error! " << error_info << ": " << strerror(error); 
}

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VL53L5CXDriver>());
  rclcpp::shutdown();
  return 0;
}
