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
    819, // PULSE_WIDTH
    409, // PHASE_SHIFT
  },
  {
    3686, // PULSE_WIDTH
    3685, // PHASE_SHIFT
  },
  {
    // Always ON
    4096, // PULSE_WIDTH
    0, // PHASE_SHIFT
  },
  {
    // Always OFF
    0, // PULSE_WIDTH
    0, // PHASE_SHIFT
  }
};
}
