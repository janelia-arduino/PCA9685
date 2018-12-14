#include <Arduino.h>
#include <PCA9685.h>


const uint8_t DEVICE_ADDRESS = 0x40;
const size_t OUTPUT_ENABLE_PIN = 2;

const size_t LOOP_DELAY = 8000;
const uint16_t FREQUENCY = 200;
const uint8_t CHANNEL = 0;

const uint8_t EXAMPLE_COUNT = 4;

const uint16_t EXAMPLE0_PULSE_WIDTH = 819;
const uint16_t EXAMPLE0_PHASE_SHIFT = 409;

const uint16_t EXAMPLE1_PULSE_WIDTH = 3686;
const uint16_t EXAMPLE1_PHASE_SHIFT = 3685;

// Always ON
const uint16_t EXAMPLE2_PULSE_WIDTH = 4096;
const uint16_t EXAMPLE2_PHASE_SHIFT = 0;

// Always OFF
const uint16_t EXAMPLE3_PULSE_WIDTH = 0;
const uint16_t EXAMPLE3_PHASE_SHIFT = 0;

uint8_t example;

uint16_t pulse_width;
uint16_t phase_shift;

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
      pulse_width = EXAMPLE0_PULSE_WIDTH;
      phase_shift = EXAMPLE0_PHASE_SHIFT;
      break;
    case 1:
      pulse_width = EXAMPLE1_PULSE_WIDTH;
      phase_shift = EXAMPLE1_PHASE_SHIFT;
      break;
    case 2:
      pulse_width = EXAMPLE2_PULSE_WIDTH;
      phase_shift = EXAMPLE2_PHASE_SHIFT;
      break;
    case 3:
      pulse_width = EXAMPLE3_PULSE_WIDTH;
      phase_shift = EXAMPLE3_PHASE_SHIFT;
      break;
  }

  pca9685.setChannelPulseWidth(CHANNEL,pulse_width,phase_shift);

  ++example;
}
