#include <Arduino.h>
#include <PCA9685.h>


const uint8_t DEVICE_ADDRESS = 0x40;
const size_t OUTPUT_ENABLE_PIN = 2;

const size_t LOOP_DELAY = 100;
const uint16_t FREQUENCY = 200;
const uint16_t TIME_INCREMENT = 100;
const uint8_t CHANNEL = 0;

uint16_t time_min;
uint16_t time_max;
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

  time_min = pca9685.getTimeMin();
  time_max = pca9685.getTimeMax();

  off_time = time_min;
  pca9685.setChannelOnTime(CHANNEL,time_min);
}

void loop()
{
  if (off_time > time_max)
  {
    off_time = time_min;
  }
  pca9685.setChannelOffTime(CHANNEL,off_time);
  off_time += TIME_INCREMENT;
  delay(LOOP_DELAY);
}
