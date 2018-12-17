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

  pca9685.setOutputsNotInverted();

  pca9685.setToFrequency(constants::frequency);

  time_min = pca9685.getTimeMin();
  time_max = pca9685.getTimeMax();

  off_time = time_min;

  pca9685.setChannelOnTime(constants::channel,time_min);
}

void loop()
{
  if (off_time > time_max)
  {
    off_time = time_min;
  }
  pca9685.setChannelOffTime(constants::channel,off_time);
  off_time += constants::time_increment;
  delay(constants::loop_delay);
}
