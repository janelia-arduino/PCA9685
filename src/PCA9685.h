// ----------------------------------------------------------------------------
// PCA9685.h
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#ifndef PCA9685_H
#define PCA9685_H
#include <Arduino.h>
#include <Wire.h>


class PCA9685
{
public:
  PCA9685();

  typedef uint16_t Channel;
  typedef uint8_t ChannelCount;
  typedef uint8_t DeviceAddress;
  typedef uint8_t DeviceIndex;
  typedef size_t Pin;
  typedef uint16_t Frequency;
  typedef double Percent;
  typedef uint16_t Time;
  typedef uint16_t Duration;
  typedef uint16_t DurationMicroseconds;

  const static Channel CHANNELS_PER_DEVICE = 16;
  enum {DEVICE_COUNT_MAX=55};

  // Convenience method when using a single device
  void setupSingleDevice(TwoWire & wire=Wire,
    DeviceAddress device_address=0x40,
    bool fast_mode_plus=false);

  // Methods for using a single device or multiple devices
  void setupOutputEnablePin(Pin output_enable_pin);
  void enableOutputs(Pin output_enable_pin);
  void disableOutputs(Pin output_enable_pin);

  Frequency getFrequencyMin();
  Frequency getFrequencyMax();
  void setToFrequency(Frequency frequency);
  Frequency getFrequency();
  void setToServoFrequency();
  Frequency getServoFrequency();

  ChannelCount getChannelCount();

  Percent getDutyCycleMin();
  Percent getDutyCycleMax();
  Percent getPercentDelayMin();
  Percent getPercentDelayMax();
  void setChannelDutyCycle(Channel channel,
    Percent duty_cycle,
    Percent percent_delay=0);
  void getChannelDutyCycle(Channel channel,
    Percent & duty_cycle,
    Percent & percent_delay);
  void setAllChannelsDutyCycle(Percent duty_cycle,
    Percent percent_delay=0);

  Duration getPulseWidthMin();
  Duration getPulseWidthMax();
  Time getPhaseShiftMin();
  Time getPhaseShiftMax();
  void setChannelPulseWidth(Channel channel,
    Duration pulse_width,
    Duration phase_shift=0);
  void getChannelPulseWidth(Channel channel,
    Duration & pulse_width,
    Time & phase_shift);
  void setAllChannelsPulseWidth(Duration pulse_width,
    Duration phase_shift=0);

  void setChannelServoPulseDuration(Channel channel,
    DurationMicroseconds pulse_duration_microseconds);
  void getChannelServoPulseDuration(Channel channel,
    DurationMicroseconds & pulse_duration_microseconds);
  void setAllChannelsServoPulseDuration(DurationMicroseconds pulse_duration_microseconds);

  Time getTimeMin();
  Time getTimeMax();
  void setChannelOnAndOffTime(Channel channel,
    Time on_time,
    Time off_time);
  void getChannelOnAndOffTime(Channel channel,
    Time & on_time,
    Time & off_time);
  void setAllChannelsOnAndOffTime(Time on_time,
    Time off_time);
  void setChannelOnTime(Channel channel,
    Time on_time);
  void getChannelOnTime(Channel channel,
    Time & on_time);
  void setAllChannelsOnTime(Time on_time);
  void setChannelOffTime(Channel channel,
    Time off_time);
  void getChannelOffTime(Channel channel,
    Time & off_time);
  void setAllChannelsOffTime(Time off_time);

  void setOutputsInverted();
  void setOutputsNotInverted();
  void setOutputsToTotemPole();
  void setOutputsToOpenDrain();
  void setOutputsLowWhenDisabled();
  void setOutputsHighWhenDisabled();
  void setOutputsHighImpedanceWhenDisabled();

  // Methods for using multiple devices
  // Take care when using fast_mode_plus with non-PCA9685 devices
  void setWire(TwoWire & wire=Wire,
    bool fast_mode_plus=false);

  // device_address=0x40 when all device address
  // hardware select lines are low
  // cannot use reserved addresses
  void addDevice(DeviceAddress device_address);
  void resetAllDevices();

  void addDeviceToGroup0(DeviceAddress device_address);
  void removeDeviceFromGroup0(DeviceAddress device_address);
  void addDeviceToGroup1(DeviceAddress device_address);
  void removeDeviceFromGroup1(DeviceAddress device_address);
  void addDeviceToGroup2(DeviceAddress device_address);
  void removeDeviceFromGroup2(DeviceAddress device_address);

  void setSingleDeviceToFrequency(DeviceAddress device_address,
    Frequency frequency);
  Frequency getSingleDeviceFrequency(DeviceAddress device_address);
  void setAllDevicesToFrequency(Frequency frequency);
  void setSingleDeviceToServoFrequency(DeviceAddress device_address);
  Frequency getSingleDeviceServoFrequency(DeviceAddress device_address);
  void setAllDevicesToServoFrequency();

  ChannelCount getDeviceChannelCount();

  // Use these device address to set more than one device at a time
  // with the methods below
  const static DeviceAddress DEVICE_ADDRESS_ALL = 0x70;
  const static DeviceAddress DEVICE_ADDRESS_GROUP0 = 0x71;
  const static DeviceAddress DEVICE_ADDRESS_GROUP1 = 0x72;
  const static DeviceAddress DEVICE_ADDRESS_GROUP2 = 0x73;

  void setDeviceChannelDutyCycle(DeviceAddress device_address,
    Channel device_channel,
    Percent duty_cycle,
    Percent percent_delay=0);
  void setAllDeviceChannelsDutyCycle(DeviceAddress device_address,
    Percent duty_cycle,
    Percent percent_delay=0);

  void setDeviceChannelPulseWidth(DeviceAddress device_address,
    Channel device_channel,
    Duration pulse_width,
    Duration phase_shift=0);
  void setAllDeviceChannelsPulseWidth(DeviceAddress device_address,
    Duration pulse_width,
    Duration phase_shift=0);

  void setDeviceChannelServoPulseDuration(DeviceAddress device_address,
    Channel device_channel,
    DurationMicroseconds pulse_duration_microseconds);
  void setAllDeviceChannelsServoPulseDuration(DeviceAddress device_address,
    DurationMicroseconds pulse_duration_microseconds);

  void setDeviceChannelOnAndOffTime(DeviceAddress device_address,
    Channel device_channel,
    Time on_time,
    Time off_time);
  void setAllDeviceChannelsOnAndOffTime(DeviceAddress device_address,
    Time on_time,
    Time off_time);
  void setDeviceChannelOnTime(DeviceAddress device_address,
    Channel device_channel,
    Time on_time);
  void setAllDeviceChannelsOnTime(DeviceAddress device_address,
    Time on_time);
  void setDeviceChannelOffTime(DeviceAddress device_address,
    Channel device_channel,
    Time off_time);
  void setAllDeviceChannelsOffTime(DeviceAddress device_address,
    Time off_time);

  void setSingleDeviceOutputsInverted(DeviceAddress device_address);
  void setAllDevicesOutputsInverted();
  void setSingleDeviceOutputsNotInverted(DeviceAddress device_address);
  void setAllDevicesOutputsNotInverted();
  void setSingleDeviceOutputsToTotemPole(DeviceAddress device_address);
  void setAllDevicesOutputsToTotemPole();
  void setSingleDeviceOutputsToOpenDrain(DeviceAddress device_address);
  void setAllDevicesOutputsToOpenDrain();
  void setSingleDeviceOutputsLowWhenDisabled(DeviceAddress device_address);
  void setAllDevicesOutputsLowWhenDisabled();
  void setSingleDeviceOutputsHighWhenDisabled(DeviceAddress device_address);
  void setAllDevicesOutputsHighWhenDisabled();
  void setSingleDeviceOutputsHighImpedanceWhenDisabled(DeviceAddress device_address);
  void setAllDevicesOutputsHighImpedanceWhenDisabled();

private:
  const static DeviceAddress DEVICE_ADDRESS_MIN = 0x40;
  const static DeviceAddress DEVICE_ADDRESS_MAX = 0x7B;
  uint8_t device_count_;
  DeviceAddress device_addresses_[DEVICE_COUNT_MAX];

  TwoWire * wire_ptr_;
  const static long FAST_MODE_PLUS_CLOCK_FREQUENCY = 1000000;

  const static int NO_OUTPUT_ENABLE_PIN = -1;

  const static int DEVICE_INDEX_NONE = -1;
  const static int DEVICE_INDEX_ALL = -2;
  const static int DEVICE_INDEX_GROUP0 = -3;
  const static int DEVICE_INDEX_GROUP1 = -4;
  const static int DEVICE_INDEX_GROUP2 = -5;
  int deviceAddressToDeviceIndex(DeviceAddress device_address);

  const static DeviceAddress GENERAL_CALL_DEVICE_ADDRESS = 0x00;
  const static uint8_t SWRST = 0b110;

  // Can write to one or more device at a time
  // so use address rather than index
  template<typename T>
  void write(DeviceAddress device_address,
    uint8_t register_address,
    T data);
  // Can only read from one device at a time
  // so use index rather than address
  template<typename T>
  void read(DeviceIndex device_index,
    uint8_t register_address,
    T & data);

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
  Mode1Register readMode1Register(DeviceIndex device_index);

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
  Mode2Register readMode2Register(DeviceIndex device_index);

  void sleep(DeviceIndex device_index);
  void wake(DeviceIndex device_index);
  void wakeAll();

  void setPrescale(DeviceIndex device_index,
    uint8_t prescale);
  void getPrescale(DeviceIndex device_index,
    uint8_t & prescale);
  uint8_t frequencyToPrescale(Frequency frequency);
  Frequency prescaleToFrequency(uint8_t prescale);

  uint8_t channelToDeviceIndex(Channel channel);
  Channel channelToDeviceChannel(Channel channel);

  void dutyCycleAndPercentDelayToPulseWidthAndPhaseShift(Percent duty_cycle,
    Percent percent_delay,
    Duration & pulse_width,
    Time & phase_shift);
  void pulseWidthAndPhaseShiftToDutyCycleAndPercentDelay(Duration pulse_width,
    Duration phase_shift,
    Percent & duty_cycle,
    Percent & percent_delay);

  void pulseWidthAndPhaseShiftToOnTimeAndOffTime(Duration pulse_width,
    Duration phase_shift,
    Time & on_time,
    Time & off_time);
  void onTimeAndOffTimeToPulseWidthAndPhaseShift(Time on_time,
    Time off_time,
    Duration & pulse_width,
    Time & phase_shift);

  void servoPulseDurationToPulseWidthAndPhaseShift(DurationMicroseconds pulse_duration_microseconds,
    Duration & pulse_width,
    Time & phase_shift);
  void pulseWidthAndPhaseShiftToServoPulseDuration(Duration pulse_width,
    Duration phase_shift,
    DurationMicroseconds & pulse_duration_microseconds);

  void setOutputsInverted(DeviceIndex device_index);
  void setOutputsNotInverted(DeviceIndex device_index);
  void setOutputsToTotemPole(DeviceIndex device_index);
  void setOutputsToOpenDrain(DeviceIndex device_index);
  void setOutputsLowWhenDisabled(DeviceIndex device_index);
  void setOutputsHighWhenDisabled(DeviceIndex device_index);
  void setOutputsHighImpedanceWhenDisabled(DeviceIndex device_index);

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
  const static uint8_t BITS_PER_TWO_BYTES = 16;
  const static uint8_t BYTE_MAX = 0xFF;
  const static uint16_t TWO_BYTE_MAX = 0xFFFF;

  const static uint8_t PRE_SCALE_REGISTER_ADDRESS = 0xFE;
  const static uint8_t PRE_SCALE_MIN = 0x03;
  const static uint8_t PRE_SCALE_MAX = 0xFF;
  // Use period instead of frequency to calculate prescale since it is linear
  // Measured 1620 Hz at prescale value 0x03, 1E6/1620=617
  // Datasheet says it should be 1526 Hz, 1E6/1526=655
  const static DurationMicroseconds PWM_PERIOD_MIN_US = 617;
  // Measured 25.3 Hz at prescale value 0xFF, 1E6/25.3=39525
  // Datasheet says it should be 24 Hz, 1E6/24=41666
  const static DurationMicroseconds PWM_PERIOD_MAX_US = 39525;
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

  const static Time TIME_MIN = 0;
  const static Time TIME_MAX = 4096;

  const static uint8_t PERCENT_MIN = 0;
  const static uint8_t PERCENT_MAX = 100;

  const static Frequency SERVO_FREQUENCY = 50;
  const static DurationMicroseconds SERVO_PERIOD_MICROSECONDS = 20000;

};

#include "PCA9685/PCA9685Definitions.h"

#endif
