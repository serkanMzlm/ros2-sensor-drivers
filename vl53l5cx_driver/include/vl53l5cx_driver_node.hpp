#ifndef __VL53L5CX_DRIVER_NODE_HPP__
#define __VL53L5CX_DRIVER_NODE_HPP__

#include <memory>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <dlfcn.h>
#include <string.h>
#include <iomanip>
#include <errno.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "vl53l5cx_api.hpp"
#include "vl53l5cx_buffers.hpp"

#define DELAY 10000

using RangeMsg = sensor_msgs::msg::Range;

class VL53L5CXDriver: public rclcpp::Node{
public:
    VL53L5CXDriver();
    int init();
    void getRange();
    void reportError(int error, std::string error_info = "Errno");
private:
    VL53L5CX_Configuration dev;
    uint8_t status, isAlive, isReady;
    VL53L5CX_ResultsData Results;

    rclcpp::Publisher<RangeMsg>::SharedPtr range_pub;
    rclcpp::TimerBase::SharedPtr timer_;
};



#endif