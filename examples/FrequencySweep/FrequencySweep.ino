#include <Arduino.h>
#include <PCA9685.h>

#include "Constants.h"


PCA9685 pca9685;

uint16_t frequency_min;
uint16_t frequency_max;
uint16_t frequency;

void setup()
{
  pca9685.setupSingleDevice(Wire,constants::device_address);

  pca9685.setupOutputEnablePin(constants::output_enable_pin);
  pca9685.enableOutputs(constants::output_enable_pin);

  pca9685.setOutputsNotInverted();

  uint16_t time_min = pca9685.getTimeMin();
  uint16_t time_max = pca9685.getTimeMax();
  pca9685.setAllChannelsOnAndOffTime(time_min,time_max/4);

  frequency_min = pca9685.getFrequencyMin();
  frequency_max = pca9685.getFrequencyMax();

  frequency = frequency_min;
}

void loop()
{
  if (frequency > frequency_max)
  {
    frequency = frequency_min;
  }
  pca9685.setToFrequency(frequency);
  frequency += constants::frequency_increment;
  delay(constants::loop_delay);
}
