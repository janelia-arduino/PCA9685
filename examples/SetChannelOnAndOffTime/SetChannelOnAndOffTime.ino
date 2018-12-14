#include <Arduino.h>
#include <PCA9685.h>


const uint8_t DEVICE_ADDRESS = 0x40;
const size_t OUTPUT_ENABLE_PIN = 2;

const size_t LOOP_DELAY = 8000;
const uint16_t FREQUENCY = 200;
const uint8_t CHANNEL = 0;

const uint8_t EXAMPLE_COUNT = 4;

// ON_TIME < OFF_TIME
const uint16_t EXAMPLE0_ON_TIME = 511;
const uint16_t EXAMPLE0_OFF_TIME = 3071;

// ON_TIME < OFF_TIME
const uint16_t EXAMPLE1_ON_TIME = 2047;
const uint16_t EXAMPLE1_OFF_TIME = 767;

// Always ON
const uint16_t EXAMPLE2_ON_TIME = 4096;
const uint16_t EXAMPLE2_OFF_TIME = 0;

// Always OFF
const uint16_t EXAMPLE3_ON_TIME = 0;
const uint16_t EXAMPLE3_OFF_TIME = 4096;

uint8_t example;

uint16_t on_time;
uint16_t off_time;

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
      on_time = EXAMPLE0_ON_TIME;
      off_time = EXAMPLE0_OFF_TIME;
      break;
    case 1:
      on_time = EXAMPLE1_ON_TIME;
      off_time = EXAMPLE1_OFF_TIME;
      break;
    case 2:
      on_time = EXAMPLE2_ON_TIME;
      off_time = EXAMPLE2_OFF_TIME;
      break;
    case 3:
      on_time = EXAMPLE3_ON_TIME;
      off_time = EXAMPLE3_OFF_TIME;
      break;
  }

  pca9685.setChannelOnAndOffTime(CHANNEL,on_time,off_time);

  ++example;
}
