#ifndef __VL53L5CX_MPU6050_DRIVER_HPP__
#define __VL53L5CX_MPU6050_DRIVER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "mpu6050_driver.hpp"
#include "vl53l5cx_driver.hpp"

using RangeMsg = sensor_msgs::msg::Range;

class MultiSensor: public rclcpp::Node{
public:
    MultiSensor();
    void getRange();
    void getIMU();
    void callbackSensor();

private:
    bool vl53l5cx_flag = true;
    bool mpu6050_flag = true;
    std::unique_ptr<VL53L5CXDriver> vl53l5cx;
    // std::unique_ptr<VL53L5CXDriver> vl53l5cx;
    rclcpp::Publisher<RangeMsg>::SharedPtr range_pub;
    rclcpp::TimerBase::SharedPtr timer_;
};



#endif
