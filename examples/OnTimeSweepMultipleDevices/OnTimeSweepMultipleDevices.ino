#include <Arduino.h>
#include <PCA9685.h>

#include "Constants.h"


PCA9685 pca9685;

PCA9685::Time time_min;
PCA9685::Time time_max;
PCA9685::Time on_time;

void setup()
{
  pca9685.setWire(Wire);
  for (PCA9685::DeviceIndex=0; device_index<constants::DEVICE_COUNT; ++device_index)
  {
    pca9685.addDevice(constants::device_addresses[device_index]);
  }
  pca9685.resetAllDevices();

  pca9685.setupOutputEnablePin(constants::output_enable_pin);
  pca9685.enableOutputs(constants::output_enable_pin);

  pca9685.setAllDevicesOutputsNotInverted();

  pca9685.setAllDevicesToFrequency(constants::frequency);

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
  pca9685.setAllDeviceChannelsOnTime(constants::device_addresses[constants::device_index],on_time);
  on_time += constants::time_increment;
  delay(constants::loop_delay);
}
