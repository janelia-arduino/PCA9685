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
