#include <Arduino.h>
#include <PCA9685.h>

#include "Constants.h"


PCA9685 pca9685;

uint8_t example_index;

void setup()
{
  pca9685.setupSingleDevice(Wire,constants::device_address);

  pca9685.setupOutputEnablePin(constants::output_enable_pin);
  pca9685.enableOutputs(constants::output_enable_pin);

  pca9685.setToFrequency(constants::frequency);

  example_index = 0;
}

void loop()
{
  delay(constants::loop_delay);
  if (example_index >= constants::EXAMPLE_COUNT)
  {
    example_index = 0;
  }
  constants::Example example = constants::examples[example_index++];

  pca9685.setChannelDutyCycle(constants::channel,example.duty_cycle,example.percent_delay);
}
