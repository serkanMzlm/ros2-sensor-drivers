#ifndef __VL53L5CX_DRIVER_HPP__
#define __VL53L5CX_DRIVER_HPP__

#include <iomanip>
#include <errno.h>
#include <iostream>
#include <unistd.h>

#include "vl53l5cx_api.hpp"
#include "vl53l5cx_buffers.hpp"

#define DELAY 10000

class VL53L5CXDriver{
public:
    VL53L5CXDriver();
    int initVL53L5CX();
    int getRange();
    void reportError(int error, std::string error_info = "Errno");

private:
    int read_data_que[4] = {5, 6, 9, 10};
    VL53L5CX_Configuration dev;
    uint8_t status, isAlive, isReady;
    VL53L5CX_ResultsData Results;
};
#endif