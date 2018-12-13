#include <Arduino.h>
#include <PCA9685.h>


const uint8_t DEVICE_ADDRESS = 0x40;
const size_t OUTPUT_ENABLE_PIN = 2;

const size_t LOOP_DELAY = 100;
const uint16_t FREQUENCY = 200;
const uint16_t TIME_INCREMENT = 100;

uint16_t time_min;
uint16_t time_max;
uint16_t on_time;

PCA9685 pca9685;

void setup()
{
  pca9685.setWire(Wire);
  pca9685.addDevice(DEVICE_ADDRESS);
  pca9685.resetAllDevices();
  pca9685.setupOutputEnablePin(OUTPUT_ENABLE_PIN);
  pca9685.enableOutputs(OUTPUT_ENABLE_PIN);
  pca9685.setAllDevicesOutputsNotInverted();

  pca9685.setAllDevicesToFrequency(FREQUENCY);

  time_min = pca9685.getTimeMin();
  time_max = pca9685.getTimeMax();

  on_time = time_min;
  pca9685.setAllDeviceChannelsOffTime(PCA9685::DEVICE_ADDRESS_ALL,time_max);
}

void loop()
{
  if (on_time > time_max)
  {
    on_time = time_min;
  }
  pca9685.setAllDeviceChannelsOnTime(PCA9685::DEVICE_ADDRESS_ALL,on_time);
  on_time += TIME_INCREMENT;
  delay(LOOP_DELAY);
}
