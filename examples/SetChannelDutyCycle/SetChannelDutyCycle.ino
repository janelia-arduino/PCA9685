#include <Arduino.h>
#include <PCA9685.h>


const uint8_t DEVICE_ADDRESS = 0x40;
const size_t OUTPUT_ENABLE_PIN = 2;

const size_t LOOP_DELAY = 8000;
const uint16_t FREQUENCY = 200;
const uint8_t CHANNEL = 0;

const uint8_t EXAMPLE_COUNT = 4;

const double EXAMPLE0_DUTY_CYCLE = 20.0;
const double EXAMPLE0_PERCENT_DELAY = 10.0;

const double EXAMPLE1_DUTY_CYCLE = 90.0;
const double EXAMPLE1_PERCENT_DELAY = 90.0;

// Always ON
const double EXAMPLE2_DUTY_CYCLE = 100.0;
const double EXAMPLE2_PERCENT_DELAY = 0.0;

// Always OFF
const double EXAMPLE3_DUTY_CYCLE = 0.0;
const double EXAMPLE3_PERCENT_DELAY = 0.0;

uint8_t example;
double duty_cycle;
double percent_delay;

PCA9685 pca9685;

void setup()
{
  pca9685.setWire(Wire);
  pca9685.addDevice(DEVICE_ADDRESS);
  pca9685.resetAllDevices();
  pca9685.setupOutputEnablePin(OUTPUT_ENABLE_PIN);
  pca9685.enableOutputs(OUTPUT_ENABLE_PIN);
  pca9685.setOneDeviceOutputsNotInverted(DEVICE_ADDRESS);

  pca9685.setOneDeviceToFrequency(DEVICE_ADDRESS,FREQUENCY);

  example = 0;
}

void loop()
{
  delay(LOOP_DELAY);
  if (example >= EXAMPLE_COUNT)
  {
    example = 0;
  }
  switch (example)
  {
    case 0:
      duty_cycle = EXAMPLE0_DUTY_CYCLE;
      percent_delay = EXAMPLE0_PERCENT_DELAY;
      break;
    case 1:
      duty_cycle = EXAMPLE1_DUTY_CYCLE;
      percent_delay = EXAMPLE1_PERCENT_DELAY;
      break;
    case 2:
      duty_cycle = EXAMPLE2_DUTY_CYCLE;
      percent_delay = EXAMPLE2_PERCENT_DELAY;
      break;
    case 3:
      duty_cycle = EXAMPLE3_DUTY_CYCLE;
      percent_delay = EXAMPLE3_PERCENT_DELAY;
      break;
  }

  pca9685.setChannelDutyCycle(CHANNEL,duty_cycle,percent_delay);

  ++example;
}
