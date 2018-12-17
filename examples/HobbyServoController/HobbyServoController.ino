#include <Arduino.h>
#include <PCA9685.h>

#include "Constants.h"


PCA9685 pca9685;

uint16_t time_min;
uint16_t time_max;
uint16_t off_time;

void setup()
{
  pca9685.setupSingleDevice(Wire,constants::device_address);

  pca9685.setupOutputEnablePin(constants::output_enable_pin);
  pca9685.enableOutputs(constants::output_enable_pin);

  pca9685.setToHobbyServoFrequency();
}

void loop()
{
  pca9685.setChannelPulseWidth(constants::channel,409);
  delay(constants::loop_delay);
}
