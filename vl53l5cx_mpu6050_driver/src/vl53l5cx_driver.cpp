#include "vl53l5cx_driver.hpp"

VL53L5CXDriver::VL53L5CXDriver(){
    // if(init() < 0){
    //     exit(-1);
    // }
}

int VL53L5CXDriver::initVL53L5CX(){
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

int VL53L5CXDriver::getRange(){
    status = vl53l5cx_check_data_ready(&dev, &isReady);
    int sum = 0;
    if(isReady){ 
        vl53l5cx_get_ranging_data(&dev, &Results);
        for(int a:read_data_que){
            sum += Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*a];
		}
        sum = sum/4;
        return sum;
    }
    return -1;
}


void VL53L5CXDriver::reportError(int error, std::string error_info ){
	std::cerr << "Error! " << error_info << ": " << strerror(error); 
}