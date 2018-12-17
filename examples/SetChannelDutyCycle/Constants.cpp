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
