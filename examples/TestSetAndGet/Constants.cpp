// ----------------------------------------------------------------------------
// Constants.cpp
//
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Constants.h"


namespace constants
{
const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;

const long baud = 115200;

const size_t loop_delay = 100;
const PCA9685::Frequency frequency = 200;
const PCA9685::Time time_increment = 400;
const PCA9685::Percent epsilon = 0.001;
}
