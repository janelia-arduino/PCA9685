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

  // Convenience method when using a single device
  void setupSingleDevice(TwoWire & wire=Wire,
    uint8_t device_address=0x40);

  // Methods for using a single device or multiple devices
  void setupOutputEnablePin(size_t output_enable_pin);
  void enableOutputs(size_t output_enable_pin);
  void disableOutputs(size_t output_enable_pin);

  uint16_t getFrequencyMin();
  uint16_t getFrequencyMax();
  void setToFrequency(uint16_t frequency);
  void setToHobbyServoFrequency();

  uint8_t getChannelCount();

  double getDutyCycleMin();
  double getDutyCycleMax();
  double getPercentDelayMin();
  double getPercentDelayMax();
  void setChannelDutyCycle(uint8_t channel,
    double duty_cycle,
    double percent_delay=0);
  void setAllChannelsDutyCycle(double duty_cycle,
    double percent_delay=0);

  uint16_t getPulseWidthMin();
  uint16_t getPulseWidthMax();
  uint16_t getPhaseShiftMin();
  uint16_t getPhaseShiftMax();
  void setChannelPulseWidth(uint8_t channel,
    uint16_t pulse_width,
    uint16_t phase_shift=0);
  void setAllChannelsPulseWidth(uint16_t pulse_width,
    uint16_t phase_shift=0);

  uint16_t getTimeMin();
  uint16_t getTimeMax();
  void setChannelOnAndOffTime(uint8_t channel,
    uint16_t on_time,
    uint16_t off_time);
  void setAllChannelsOnAndOffTime(uint16_t on_time,
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

  // Methods for using multiple devices
  void setWire(TwoWire & wire=Wire);

  // device_address=0x40 when all device address
  // hardware select lines are low
  // cannot use reserved addresses
  void addDevice(uint8_t device_address);
  void resetAllDevices();

  const static uint8_t DEVICE_ADDRESS_ALL = 0x70;
  const static uint8_t DEVICE_ADDRESS_GROUP0 = 0x71;
  const static uint8_t DEVICE_ADDRESS_GROUP1 = 0x72;
  const static uint8_t DEVICE_ADDRESS_GROUP2 = 0x73;
  void addDeviceToGroup0(uint8_t device_address);
  void removeDeviceFromGroup0(uint8_t device_address);
  void addDeviceToGroup1(uint8_t device_address);
  void removeDeviceFromGroup1(uint8_t device_address);
  void addDeviceToGroup2(uint8_t device_address);
  void removeDeviceFromGroup2(uint8_t device_address);

  void setSingleDeviceToFrequency(uint8_t device_address,
    uint16_t frequency);
  void setAllDevicesToFrequency(uint16_t frequency);
  void setSingleDeviceToHobbyServoFrequency(uint8_t device_address);
  void setAllDevicesToHobbyServoFrequency();

  uint8_t getDeviceChannelCount();

  void setDeviceChannelDutyCycle(uint8_t device_address,
    uint8_t device_channel,
    double duty_cycle,
    double percent_delay=0);
  void setAllDeviceChannelsDutyCycle(uint8_t device_address,
    double duty_cycle,
    double percent_delay=0);

  void setDeviceChannelPulseWidth(uint8_t device_address,
    uint8_t device_channel,
    uint16_t pulse_width,
    uint16_t phase_shift=0);
  void setAllDeviceChannelsPulseWidth(uint8_t device_address,
    uint16_t pulse_width,
    uint16_t phase_shift=0);

  void setDeviceChannelOnAndOffTime(uint8_t device_address,
    uint8_t device_channel,
    uint16_t on_time,
    uint16_t off_time);
  void setAllDeviceChannelsOnAndOffTime(uint8_t device_address,
    uint16_t on_time,
    uint16_t off_time);
  void setDeviceChannelOnTime(uint8_t device_address,
    uint8_t device_channel,
    uint16_t on_time);
  void setAllDeviceChannelsOnTime(uint8_t device_address,
    uint16_t on_time);
  void setDeviceChannelOffTime(uint8_t device_address,
    uint8_t device_channel,
    uint16_t off_time);
  void setAllDeviceChannelsOffTime(uint8_t device_address,
    uint16_t off_time);

  void setSingleDeviceOutputsInverted(uint8_t device_address);
  void setAllDevicesOutputsInverted();
  void setSingleDeviceOutputsNotInverted(uint8_t device_address);
  void setAllDevicesOutputsNotInverted();
  void setSingleDeviceOutputsToTotemPole(uint8_t device_address);
  void setAllDevicesOutputsToTotemPole();
  void setSingleDeviceOutputsToOpenDrain(uint8_t device_address);
  void setAllDevicesOutputsToOpenDrain();
  void setSingleDeviceOutputsLowWhenDisabled(uint8_t device_address);
  void setAllDevicesOutputsLowWhenDisabled();
  void setSingleDeviceOutputsHighWhenDisabled(uint8_t device_address);
  void setAllDevicesOutputsHighWhenDisabled();
  void setSingleDeviceOutputsHighImpedanceWhenDisabled(uint8_t device_address);
  void setAllDevicesOutputsHighImpedanceWhenDisabled();

private:
  const static uint8_t DEVICE_ADDRESS_MIN = 0x40;
  const static uint8_t DEVICE_ADDRESS_MAX = 0x7B;
  enum {DEVICE_COUNT_MAX=55};
  uint8_t device_count_;
  uint8_t device_addresses_[DEVICE_COUNT_MAX];

  TwoWire * wire_ptr_;

  const static int NO_OUTPUT_ENABLE_PIN = -1;
  const static uint8_t CHANNELS_PER_DEVICE = 16;

  const static uint8_t DEVICE_INDEX_NONE = -1;
  const static uint8_t DEVICE_INDEX_ALL = -2;
  const static uint8_t DEVICE_INDEX_GROUP0 = -3;
  const static uint8_t DEVICE_INDEX_GROUP1 = -4;
  const static uint8_t DEVICE_INDEX_GROUP2 = -5;
  int deviceAddressToDeviceIndex(uint8_t device_address);

  const static uint8_t GENERAL_CALL_DEVICE_ADDRESS = 0x00;
  const static uint8_t SWRST = 0b110;

  const static uint8_t READ_BYTE_COUNT = 1;

  // Can write to one or more device at a time
  // so use address rather than index
  void write(uint8_t device_address,
    uint8_t register_address,
    uint8_t data);
  // Can only read from one device at a time
  // so use index rather than address
  uint8_t read(uint8_t device_index,
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
  Mode1Register readMode1Register(uint8_t device_index);

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
  Mode2Register readMode2Register(uint8_t device_index);

  void sleep(uint8_t device_index);
  void wake(uint8_t device_index);
  void wakeAll();

  uint8_t frequencyToPrescale(uint16_t frequency);
  void setPrescale(uint8_t device_index,
    uint8_t prescale);

  uint8_t channelToDeviceIndex(uint8_t channel);
  uint8_t channelToDeviceChannel(uint8_t channel);

  void dutyCycleAndPercentDelayToPulseWidthAndPhaseShift(double duty_cycle,
    double percent_delay,
    uint16_t & pulse_width,
    uint16_t & phase_shift);

  void pulseWidthAndPhaseShiftToOnTimeAndOffTime(uint16_t pulse_width,
    uint16_t phase_shift,
    uint16_t & on_time,
    uint16_t & off_time);

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

  void setOutputsInverted(uint8_t device_index);
  void setOutputsNotInverted(uint8_t device_index);
  void setOutputsToTotemPole(uint8_t device_index);
  void setOutputsToOpenDrain(uint8_t device_index);
  void setOutputsLowWhenDisabled(uint8_t device_index);
  void setOutputsHighWhenDisabled(uint8_t device_index);
  void setOutputsHighImpedanceWhenDisabled(uint8_t device_index);

  const static uint8_t DOES_NOT_RESPOND = 0;
  const static uint8_t DOES_RESPOND = 1;
  const static uint8_t SUBADR1_REGISTER_ADDRESS = 0x02;
  const static uint8_t SUBADR2_REGISTER_ADDRESS = 0x03;
  const static uint8_t SUBADR3_REGISTER_ADDRESS = 0x04;
  const static uint8_t ALLCALLADR_REGISTER_ADDRESS = 0x05;

  const static uint8_t LED0_ON_L_REGISTER_ADDRESS = 0x06;
  const static uint8_t LED0_OFF_L_REGISTER_ADDRESS = 0x08;
  const static uint8_t ALL_LED_ON_L_REGISTER_ADDRESS = 0xFA;
  const static uint8_t ALL_LED_OFF_L_REGISTER_ADDRESS = 0xFC;
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
  const static uint16_t TIME_MAX = 4096;

  const static uint8_t PERCENT_MIN = 0;
  const static uint8_t PERCENT_MAX = 100;

  const static uint16_t HOBBY_SERVO_FREQUENCY = 50;
};

#endif
