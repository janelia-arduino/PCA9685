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

const size_t loop_delay = 8000;
const PCA9685::Frequency frequency = 200;
const PCA9685::Channel channel = 0;

const Example examples[EXAMPLE_COUNT] =
{
  {
    20.0, // DUTY_CYCLE
    10.0, // PERCENT_DELAY
  },
  {
    90.0, // DUTY_CYCLE
    90.0, // PERCENT_DELAY
  },
  {
    // Always ON
    100.0, // DUTY_CYCLE
    0.0, // PERCENT_DELAY
  },
  {
    // Always OFF
    0.0, // DUTY_CYCLE
    0.0, // PERCENT_DELAY
  }
};
}
