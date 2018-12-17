// ----------------------------------------------------------------------------
// Constants.cpp
//
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#include "Constants.h"


namespace constants
{
const uint8_t device_addresses[DEVICE_COUNT] =
{
  0x40,
  0x41,
  0x42
};
const uint8_t device_index = 0;

const size_t output_enable_pin = 2;

const size_t loop_delay = 100;
const uint16_t frequency = 200;
const uint16_t time_increment = 100;

const uint8_t channel = 0;
}
