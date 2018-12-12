// ----------------------------------------------------------------------------
// PCA9685.h
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#ifndef PCA9685_H
#define PCA9685_H
#include <Arduino.h>
#include <Wire.h>


class PCA9685
{
public:
  PCA9685();

  void setWire(TwoWire & wire=Wire);

  uint8_t addDevice(uint8_t device_address);
  void resetAllDevices();

  void setOutputEnablePin(uint8_t device_index,
    size_t pin);
  void setAllOutputEnablePins(size_t pin);
  void enableOutputs(uint8_t device_index);
  void enableAllOutputs();
  void disableOutputs(uint8_t device_index);
  void disableAllOutputs();

  uint16_t getFrequencyMin();
  uint16_t getFrequencyMax();
  void setFrequency(uint8_t device_index,
    uint16_t frequency);
  void setAllFrequencies(uint16_t frequency);

  uint16_t getTimeMin();
  uint16_t getTimeMax();
  void setOnAndOffTime(uint8_t channel,
    uint16_t on_time,
    uint16_t off_time);
  void setAllChannelsOnAndOffTimes(uint16_t on_time,
    uint16_t off_time);
  void setChannelOnTime(uint8_t channel,
    uint16_t on_time);
  void setAllChannelsOnTime(uint16_t on_time);
  void setChannelOffTime(uint8_t channel,
    uint16_t off_time);
  void setAllChannelsOffTime(uint16_t off_time);

  void setOutputsInverted();
  void setOutputsNotInverted();
  void setOutputsToTotemPole();
  void setOutputsToOpenDrain();
  void setOutputsLowWhenDisabled();
  void setOutputsHighWhenDisabled();
  void setOutputsHighImpedanceWhenDisabled();

private:
  TwoWire * wire_ptr_;

  enum {DEVICE_COUNT_MAX=62};
  uint8_t device_addresses_[DEVICE_COUNT_MAX];
  uint8_t device_count_;

  const static uint8_t CHANNELS_PER_DEVICE = 16;
  uint8_t channel_count_;

  const static int NO_OUTPUT_ENABLE_PIN = -1;
  int output_enable_pins_[DEVICE_COUNT_MAX];

  const static uint8_t GENERAL_CALL_DEVICE_ADDRESS = 0x00;
  const static uint8_t SWRST = 0b110;

  const static uint8_t READ_BYTE_COUNT = 1;

  void writeByte(uint8_t device_address,
    uint8_t register_address,
    uint8_t data);
  uint8_t readByte(uint8_t device_address,
    uint8_t register_address);

  const static uint8_t MODE1_REGISTER_ADDRESS = 0x00;
  union Mode1Register
  {
    struct
    {
      uint8_t allcall : 1;
      uint8_t sub3 : 1;
      uint8_t sub2 : 1;
      uint8_t sub1 : 1;
      uint8_t sleep : 1;
      uint8_t ai : 1;
      uint8_t extclk : 1;
      uint8_t restart : 1;
    } fields;
    uint8_t data;
  };
  Mode1Register readMode1Register();

  const static uint8_t MODE2_REGISTER_ADDRESS = 0x01;
  union Mode2Register
  {
    struct
    {
      uint8_t outne : 2;
      uint8_t outdrv : 1;
      uint8_t och : 1;
      uint8_t invrt : 1;
      uint8_t space : 3;
    } fields;
    uint8_t data;
  };
  Mode2Register readMode2Register();

  void sleep();
  void wake();

  uint8_t frequencyToPrescale(uint16_t frequency);
  void setPrescale(uint8_t prescale);
  uint8_t channelToDeviceIndex(uint8_t channel);
  void setOnAndOffTimeByRegister(uint8_t device_address,
    uint8_t register_address,
    uint16_t on_time,
    uint16_t off_time);
  void setOnTimeByRegister(uint8_t device_address,
    uint8_t register_address,
    uint16_t on_time);
  void setOffTimeByRegister(uint8_t device_address,
    uint8_t register_address,
    uint16_t off_time);

  const static uint8_t SUBADR1_REGISTER_ADDRESS = 0x02;
  const static uint8_t SUBADR2_REGISTER_ADDRESS = 0x03;
  const static uint8_t SUBADR3_REGISTER_ADDRESS = 0x04;
  const static uint8_t ALLCALLADR_REGISTER_ADDRESS = 0x05;

  const static uint8_t LED0_ON_L_REGISTER_ADDRESS = 0x06;
  const static uint8_t ALL_LED_ON_L_REGISTER_ADDRESS = 0xFA;
  const static uint8_t LED_REGISTERS_SIZE = 4;
  const static uint8_t BITS_PER_BYTE = 8;

  const static uint8_t PRE_SCALE_REGISTER_ADDRESS = 0xFE;
  const static uint8_t PRE_SCALE_MIN = 0x03;
  const static uint8_t PRE_SCALE_MAX = 0xFF;
  // Use period instead of frequency to calculate prescale since it is linear
  // Measured 1620 Hz at prescale value 0x03, 1E6/1620=617
  // Datasheet says it should be 1526 Hz, 1E6/1526=655
  const static uint16_t PWM_PERIOD_MIN_US = 617;
  // Measured 25.3 Hz at prescale value 0xFF, 1E6/25.3=39525
  // Datasheet says it should be 24 Hz, 1E6/24=41666
  const static uint16_t PWM_PERIOD_MAX_US = 39525;
  const static uint32_t MICROSECONDS_PER_SECOND = 1000000;

  const static uint8_t OUTPUTS_INVERTED = 1;
  const static uint8_t OUTPUTS_NOT_INVERTED = 0;
  const static uint8_t OUTPUTS_TOTEM_POLE = 1;
  const static uint8_t OUTPUTS_OPEN_DRAIN = 0;
  const static uint8_t OUTPUTS_LOW_WHEN_DISABLED = 0b00;
  const static uint8_t OUTPUTS_HIGH_WHEN_DISABLED = 0b01;
  const static uint8_t OUTPUTS_HIGH_IMPEDANCE_WHEN_DISABLED = 0b10;

  const static uint8_t SLEEP = 1;
  const static uint8_t WAKE = 0;
  const static uint8_t AUTO_INCREMENT_ENABLED = 1;
  const static uint8_t AUTO_INCREMENT_DISABLED = 0;
  const static uint8_t RESTART_ENABLED = 1;
  const static uint8_t RESTART_DISABLED = 0;
  const static uint8_t RESTART_CLEAR = 1;

  const static uint16_t TIME_MIN = 0;
  const static uint16_t TIME_MAX = 4095;
};

#endif
