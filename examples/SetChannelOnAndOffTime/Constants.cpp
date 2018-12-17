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
const uint8_t device_address = 0x40;
const size_t output_enable_pin = 2;

const size_t loop_delay = 8000;
const uint16_t frequency = 200;
const uint8_t channel = 0;

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
