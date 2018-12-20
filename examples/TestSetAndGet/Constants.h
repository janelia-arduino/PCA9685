// ----------------------------------------------------------------------------
// Constants.h
//
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <Arduino.h>


namespace constants
{
extern const uint8_t device_address;
extern const size_t output_enable_pin;

extern const long baud;
extern const size_t loop_delay;
extern const uint16_t frequency;
extern const uint16_t time_increment;
extern const double epsilon;
}
#endif
