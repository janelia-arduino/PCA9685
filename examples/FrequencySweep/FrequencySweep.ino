#include <Arduino.h>
#include <PCA9685.h>


const uint8_t DEVICE_ADDRESS = 0x40;
const size_t OUTPUT_ENABLE_PIN = 2;

const size_t LOOP_DELAY = 100;
const uint16_t FREQUENCY_INCREMENT = 10;

uint8_t device_index;
uint16_t frequency_min;
uint16_t frequency_max;
uint16_t frequency;

PCA9685 pca9685;

void setup()
{
  pca9685.setWire(Wire);
  device_index = pca9685.addDevice(DEVICE_ADDRESS);
  pca9685.setOutputEnablePin(device_index,OUTPUT_ENABLE_PIN);
  pca9685.enableOutputs(device_index);

  uint16_t time_min = pca9685.getTimeMin();
  uint16_t time_max = pca9685.getTimeMax();
  pca9685.setAllChannelsOnAndOffTimes(time_min,time_max/4);

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
  pca9685.setFrequency(device_index,frequency);
  frequency += FREQUENCY_INCREMENT;
  delay(LOOP_DELAY);
}
