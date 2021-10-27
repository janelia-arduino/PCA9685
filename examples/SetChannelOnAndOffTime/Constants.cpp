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
    // ON_TIME < OFF_TIME
    511, // ON_TIME
    3071, // OFF_TIME
  },
  {
    // ON_TIME < OFF_TIME
    2047, // ON_TIME
    767, // OFF_TIME
  },
  {
    // Always ON
    4096, // ON_TIME
    0, // OFF_TIME
  },
  {
    // Always OFF
    0, // ON_TIME
    4096, // OFF_TIME
  }
};
}
