#include "vl53l5cx_mpu6050_driver.hpp"


MultiSensor::MultiSensor(): Node("multi_sensor"), 
   vl53l5cx{std::make_unique<VL53L5CXDriver>()} 
{
    range_pub = this->create_publisher<RangeMsg>("range", 10);
    if(vl53l5cx->initVL53L5CX() < 0){
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize VL53L5CX...");
        vl53l5cx_flag = false;
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(
                        &MultiSensor::callbackSensor, this));
}


void MultiSensor::getRange(){
    RangeMsg msg;
    msg.range = vl53l5cx->getRange();
    if(msg.range >= 0){
        range_pub->publish(msg);
    }
}

void MultiSensor::getIMU(){

}

void MultiSensor::callbackSensor(){
    if(vl53l5cx_flag){
        getRange();
    }
}



int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiSensor>());
    rclcpp::shutdown();
    return 0;
}