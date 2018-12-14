// ----------------------------------------------------------------------------
// PCA9685.cpp
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#include "PCA9685.h"


PCA9685::PCA9685()
{
  device_count_ = 0;
  for (uint8_t device_index=0; device_index<DEVICE_COUNT_MAX; ++device_index)
  {
    device_addresses_[device_index] = GENERAL_CALL_DEVICE_ADDRESS;
  }
}

void PCA9685::setWire(TwoWire & wire)
{
  wire_ptr_ = &wire;
  wire_ptr_->begin();
}

void PCA9685::addDevice(uint8_t device_address)
{
  if ((device_address < DEVICE_ADDRESS_MIN) ||
    (device_address > DEVICE_ADDRESS_MAX) ||
    (device_address == DEVICE_ADDRESS_ALL) ||
    (device_address == DEVICE_ADDRESS_GROUP0) ||
    (device_address == DEVICE_ADDRESS_GROUP1) ||
    (device_address == DEVICE_ADDRESS_GROUP2))
  {
    return;
  }
  device_addresses_[device_count_++] = device_address;
}

void PCA9685::resetAllDevices()
{
  wire_ptr_->beginTransmission(GENERAL_CALL_DEVICE_ADDRESS);
  wire_ptr_->write(SWRST);
  wire_ptr_->endTransmission();
  delay(10);
  wakeAll();
}

void PCA9685::setupOutputEnablePin(size_t output_enable_pin)
{
  pinMode(output_enable_pin,OUTPUT);
  digitalWrite(output_enable_pin,HIGH);
}

void PCA9685::enableOutputs(size_t output_enable_pin)
{
  digitalWrite(output_enable_pin,LOW);
}

void PCA9685::disableOutputs(size_t output_enable_pin)
{
  digitalWrite(output_enable_pin,HIGH);
}

void PCA9685::addDeviceToGroup0(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub1 = DOES_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::removeDeviceFromGroup0(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub1 = DOES_NOT_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::addDeviceToGroup1(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub2 = DOES_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::removeDeviceFromGroup1(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub2 = DOES_NOT_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::addDeviceToGroup2(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub3 = DOES_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::removeDeviceFromGroup2(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub3 = DOES_NOT_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

uint16_t PCA9685::getFrequencyMin()
{
  return MICROSECONDS_PER_SECOND / PWM_PERIOD_MAX_US;
}

uint16_t PCA9685::getFrequencyMax()
{
  return MICROSECONDS_PER_SECOND / PWM_PERIOD_MIN_US;
}

void PCA9685::setOneDeviceToFrequency(uint8_t device_address,
  uint16_t frequency)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  uint8_t prescale = frequencyToPrescale(frequency);
  setPrescale(device_index,prescale);
}

void PCA9685::setAllDevicesToFrequency(uint16_t frequency)
{
  uint8_t prescale = frequencyToPrescale(frequency);
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    setPrescale(device_index,prescale);
  }
}

uint8_t PCA9685::getChannelCount()
{
  return CHANNELS_PER_DEVICE * device_count_;
}

uint8_t PCA9685::getDeviceChannelCount()
{
  return CHANNELS_PER_DEVICE;
}

double PCA9685::getDutyCycleMin()
{
  return PERCENT_MIN;
}

double PCA9685::getDutyCycleMax()
{
  return PERCENT_MAX;
}

double PCA9685::getPercentDelayMin()
{
  return PERCENT_MIN;
}

double PCA9685::getPercentDelayMax()
{
  return PERCENT_MAX;
}

void PCA9685::setChannelDutyCycle(uint8_t channel,
    double duty_cycle,
    double percent_delay)
{
  uint16_t pulse_width;
  uint16_t phase_shift;
  dutyCycleAndPercentDelayToPulseWidthAndPhaseShift(duty_cycle,percent_delay,pulse_width,phase_shift);
  setChannelPulseWidth(channel,pulse_width,phase_shift);
}

void PCA9685::setDeviceChannelDutyCycle(uint8_t device_address,
    uint8_t device_channel,
    double duty_cycle,
    double percent_delay)
{
  uint16_t pulse_width;
  uint16_t phase_shift;
  dutyCycleAndPercentDelayToPulseWidthAndPhaseShift(duty_cycle,percent_delay,pulse_width,phase_shift);
  setDeviceChannelPulseWidth(device_address,device_channel,pulse_width,phase_shift);
}

void PCA9685::setAllDeviceChannelsDutyCycle(uint8_t device_address,
    double duty_cycle,
    double percent_delay)
{
  uint16_t pulse_width;
  uint16_t phase_shift;
  dutyCycleAndPercentDelayToPulseWidthAndPhaseShift(duty_cycle,percent_delay,pulse_width,phase_shift);
  setAllDeviceChannelsPulseWidth(device_address,pulse_width,phase_shift);
}

uint16_t PCA9685::getPulseWidthMin()
{
  return TIME_MIN;
}

uint16_t PCA9685::getPulseWidthMax()
{
  return TIME_MAX;
}

uint16_t PCA9685::getPhaseShiftMin()
{
  return TIME_MIN;
}

uint16_t PCA9685::getPhaseShiftMax()
{
  return TIME_MAX - 1;
}

void PCA9685::setChannelPulseWidth(uint8_t channel,
    uint16_t pulse_width,
    uint16_t phase_shift)
{
  uint16_t on_time;
  uint16_t off_time;
  pulseWidthAndPhaseShiftToOnTimeAndOffTime(pulse_width,phase_shift,on_time,off_time);
  Serial << "pulse_width: " << pulse_width << "\n";
  Serial << "phase_shift: " << phase_shift << "\n";
  Serial << "on_time: " << on_time << "\n";
  Serial << "off_time: " << off_time << "\n";
  setChannelOnAndOffTime(channel,on_time,off_time);
}

void PCA9685::setDeviceChannelPulseWidth(uint8_t device_address,
    uint8_t device_channel,
    uint16_t pulse_width,
    uint16_t phase_shift)
{
  uint16_t on_time;
  uint16_t off_time;
  pulseWidthAndPhaseShiftToOnTimeAndOffTime(pulse_width,phase_shift,on_time,off_time);
  setDeviceChannelOnAndOffTime(device_address,device_channel,on_time,off_time);
}

void PCA9685::setAllDeviceChannelsPulseWidth(uint8_t device_address,
    uint16_t pulse_width,
    uint16_t phase_shift)
{
  uint16_t on_time;
  uint16_t off_time;
  pulseWidthAndPhaseShiftToOnTimeAndOffTime(pulse_width,phase_shift,on_time,off_time);
  setAllDeviceChannelsOnAndOffTime(device_address,on_time,off_time);
}

uint16_t PCA9685::getTimeMin()
{
  return TIME_MIN;
}

uint16_t PCA9685::getTimeMax()
{
  return TIME_MAX;
}

void PCA9685::setChannelOnAndOffTime(uint8_t channel,
  uint16_t on_time,
  uint16_t off_time)
{
  if (channel >= getChannelCount())
  {
    return;
  }
  uint8_t device_index = channelToDeviceIndex(channel);
  uint8_t device_channel = channelToDeviceChannel(channel);
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  setOnAndOffTimeByRegister(device_addresses_[device_index],register_address,on_time,off_time);
}

void PCA9685::setDeviceChannelOnAndOffTime(uint8_t device_address,
  uint8_t device_channel,
  uint16_t on_time,
  uint16_t off_time)
{
  if (device_channel >= getDeviceChannelCount())
  {
    return;
  }
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  setOnAndOffTimeByRegister(device_address,register_address,on_time,off_time);
}

void PCA9685::setAllDeviceChannelsOnAndOffTime(uint8_t device_address,
  uint16_t on_time,
  uint16_t off_time)
{
  uint8_t register_address = ALL_LED_ON_L_REGISTER_ADDRESS;
  setOnAndOffTimeByRegister(device_address,register_address,on_time,off_time);
}

void PCA9685::setChannelOnTime(uint8_t channel,
  uint16_t on_time)
{
  if (channel >= getChannelCount())
  {
    return;
  }
  uint8_t device_index = channelToDeviceIndex(channel);
  uint8_t device_channel = channelToDeviceChannel(channel);
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  setOnTimeByRegister(device_addresses_[device_index],register_address,on_time);
}

void PCA9685::setDeviceChannelOnTime(uint8_t device_address,
  uint8_t device_channel,
  uint16_t on_time)
{
  if (device_channel >= getDeviceChannelCount())
  {
    return;
  }
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  setOnTimeByRegister(device_address,register_address,on_time);
}

void PCA9685::setAllDeviceChannelsOnTime(uint8_t device_address,
  uint16_t on_time)
{
  uint8_t register_address = ALL_LED_ON_L_REGISTER_ADDRESS;
  setOnTimeByRegister(device_address,register_address,on_time);
}

void PCA9685::setChannelOffTime(uint8_t channel,
  uint16_t off_time)
{
  if (channel >= getChannelCount())
  {
    return;
  }
  uint8_t device_index = channelToDeviceIndex(channel);
  uint8_t device_channel = channelToDeviceChannel(channel);
  uint8_t register_address = LED0_OFF_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  setOffTimeByRegister(device_addresses_[device_index],register_address,off_time);
}

void PCA9685::setDeviceChannelOffTime(uint8_t device_address,
  uint8_t device_channel,
  uint16_t off_time)
{
  if (device_channel >= getDeviceChannelCount())
  {
    return;
  }
  uint8_t register_address = LED0_OFF_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  setOffTimeByRegister(device_address,register_address,off_time);
}

void PCA9685::setAllDeviceChannelsOffTime(uint8_t device_address,
  uint16_t off_time)
{
  uint8_t register_address = ALL_LED_OFF_L_REGISTER_ADDRESS;
  setOffTimeByRegister(device_address,register_address,off_time);
}

void PCA9685::setOneDeviceOutputsInverted(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsInverted(device_index);
}

void PCA9685::setAllDevicesOutputsInverted()
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsInverted(device_index);
  }
}

void PCA9685::setOneDeviceOutputsNotInverted(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsNotInverted(device_index);
}

void PCA9685::setAllDevicesOutputsNotInverted()
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsNotInverted(device_index);
  }
}

void PCA9685::setOneDeviceOutputsToTotemPole(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsToTotemPole(device_index);
}

void PCA9685::setAllDevicesOutputsToTotemPole()
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsToTotemPole(device_index);
  }
}

void PCA9685::setOneDeviceOutputsToOpenDrain(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsToOpenDrain(device_index);
}

void PCA9685::setAllDevicesOutputsToOpenDrain()
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsToOpenDrain(device_index);
  }
}

void PCA9685::setOneDeviceOutputsLowWhenDisabled(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsLowWhenDisabled(device_index);
}

void PCA9685::setAllDevicesOutputsLowWhenDisabled()
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsLowWhenDisabled(device_index);
  }
}

void PCA9685::setOneDeviceOutputsHighWhenDisabled(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsHighWhenDisabled(device_index);
}

void PCA9685::setAllDevicesOutputsHighWhenDisabled()
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsHighWhenDisabled(device_index);
  }
}

void PCA9685::setOneDeviceOutputsHighImpedanceWhenDisabled(uint8_t device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsHighImpedanceWhenDisabled(device_index);
}

void PCA9685::setAllDevicesOutputsHighImpedanceWhenDisabled()
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsHighImpedanceWhenDisabled(device_index);
  }
}

// private

int PCA9685::deviceAddressToDeviceIndex(uint8_t device_address)
{
  uint8_t device_index = DEVICE_INDEX_NONE;
  if (device_address == DEVICE_ADDRESS_ALL)
  {
    device_index = DEVICE_INDEX_ALL;
  }
  else if (device_address == DEVICE_ADDRESS_GROUP0)
  {
    device_index = DEVICE_INDEX_GROUP0;
  }
  else if (device_address == DEVICE_ADDRESS_GROUP1)
  {
    device_index = DEVICE_INDEX_GROUP1;
  }
  else if (device_address == DEVICE_ADDRESS_GROUP2)
  {
    device_index = DEVICE_INDEX_GROUP2;
  }
  else
  {
    for (uint8_t index=0; index<device_count_; ++index)
    {
      if (device_address == device_addresses_[index])
      {
        device_index = index;
        break;
      }
    }
  }
  return device_index;
}

void PCA9685::write(uint8_t device_address,
  uint8_t register_address,
  uint8_t data)
{
  wire_ptr_->beginTransmission(device_address);
  wire_ptr_->write(register_address);
  wire_ptr_->write(data);
  wire_ptr_->endTransmission();
}

uint8_t PCA9685::read(uint8_t device_index,
  uint8_t register_address)
{
  uint8_t device_address = device_addresses_[device_index];
  wire_ptr_->beginTransmission(device_address);
  wire_ptr_->write(register_address);
  wire_ptr_->endTransmission();

  wire_ptr_->requestFrom(device_address,READ_BYTE_COUNT);
  return wire_ptr_->read();
}

PCA9685::Mode1Register PCA9685::readMode1Register(uint8_t device_index)
{
  Mode1Register mode1_register;
  mode1_register.data = read(device_index,MODE1_REGISTER_ADDRESS);
  // Serial << "restart: " << mode1_register.fields.restart << "\n";
  // Serial << "extclk: " << mode1_register.fields.extclk << "\n";
  // Serial << "ai: " << mode1_register.fields.ai << "\n";
  // Serial << "sleep: " << mode1_register.fields.sleep << "\n";
  // Serial << "sub1: " << mode1_register.fields.sub1 << "\n";
  // Serial << "sub2: " << mode1_register.fields.sub2 << "\n";
  // Serial << "sub3: " << mode1_register.fields.sub3 << "\n";
  // Serial << "allcall: " << mode1_register.fields.allcall << "\n";
  return mode1_register;
}

PCA9685::Mode2Register PCA9685::readMode2Register(uint8_t device_index)
{
  Mode2Register mode2_register;
  mode2_register.data = read(device_index,MODE2_REGISTER_ADDRESS);
  // Serial << "outne: " << mode2_register.fields.outne << "\n";
  // Serial << "outdrv: " << mode2_register.fields.outdrv << "\n";
  // Serial << "och: " << mode2_register.fields.och << "\n";
  // Serial << "invrt: " << mode2_register.fields.invrt << "\n";
  return mode2_register;
}

void PCA9685::sleep(uint8_t device_index)
{
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sleep = SLEEP;
  write(device_addresses_[device_index],MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::wake(uint8_t device_index)
{
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sleep = WAKE;
  mode1_register.fields.ai = AUTO_INCREMENT_ENABLED;
  write(device_addresses_[device_index],MODE1_REGISTER_ADDRESS,mode1_register.data);
  if (mode1_register.fields.restart == RESTART_ENABLED)
  {
    delay(1);
    mode1_register.fields.restart = RESTART_CLEAR;
    write(device_addresses_[device_index],MODE1_REGISTER_ADDRESS,mode1_register.data);
  }
}

void PCA9685::wakeAll()
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    wake(device_index);
  }
}

uint8_t PCA9685::frequencyToPrescale(uint16_t frequency)
{
  uint16_t period_us = MICROSECONDS_PER_SECOND / frequency;
  period_us = constrain(period_us,PWM_PERIOD_MIN_US,PWM_PERIOD_MAX_US);
  uint8_t prescale = map(period_us,PWM_PERIOD_MIN_US,PWM_PERIOD_MAX_US,PRE_SCALE_MIN,PRE_SCALE_MAX);
  return prescale;
}

void PCA9685::setPrescale(uint8_t device_index,
  uint8_t prescale)
{
  sleep(device_index);
  write(device_addresses_[device_index],PRE_SCALE_REGISTER_ADDRESS,prescale);
  wake(device_index);
}

uint8_t PCA9685::channelToDeviceIndex(uint8_t channel)
{
  return channel / CHANNELS_PER_DEVICE;
}

uint8_t PCA9685::channelToDeviceChannel(uint8_t channel)
{
  return channel % CHANNELS_PER_DEVICE;
}

void PCA9685::dutyCycleAndPercentDelayToPulseWidthAndPhaseShift(double duty_cycle,
  double percent_delay,
  uint16_t & pulse_width,
  uint16_t & phase_shift)
{
  pulse_width = round(((double)TIME_MAX * duty_cycle) / (double)PERCENT_MAX);
  phase_shift = round(((double)TIME_MAX * percent_delay) / (double)PERCENT_MAX);
  // datasheet says to subtract 1
  if (phase_shift >= 1)
  {
    phase_shift -= 1;
  }
}

void PCA9685::pulseWidthAndPhaseShiftToOnTimeAndOffTime(uint16_t pulse_width,
  uint16_t phase_shift,
  uint16_t & on_time,
  uint16_t & off_time)
{
  if (pulse_width == 0)
  {
    on_time = TIME_MIN;
    off_time = TIME_MAX;
    return;
  }
  else if (pulse_width >= TIME_MAX)
  {
    on_time = TIME_MAX;
    off_time = TIME_MIN;
    return;
  }
  on_time = constrain(phase_shift,getPhaseShiftMin(),getPhaseShiftMax());
  off_time = (on_time + pulse_width) % TIME_MAX;
}

void PCA9685::setOnAndOffTimeByRegister(uint8_t device_address,
  uint8_t register_address,
  uint16_t on_time,
  uint16_t off_time)
{
  wire_ptr_->beginTransmission(device_address);
  wire_ptr_->write(register_address);
  wire_ptr_->write(on_time);
  wire_ptr_->write(on_time >> BITS_PER_BYTE);
  wire_ptr_->write(off_time);
  wire_ptr_->write(off_time >> BITS_PER_BYTE);
  wire_ptr_->endTransmission();
}

void PCA9685::setOnTimeByRegister(uint8_t device_address,
  uint8_t register_address,
  uint16_t on_time)
{
  wire_ptr_->beginTransmission(device_address);
  wire_ptr_->write(register_address);
  wire_ptr_->write(on_time);
  wire_ptr_->write(on_time >> BITS_PER_BYTE);
  wire_ptr_->endTransmission();
}

void PCA9685::setOffTimeByRegister(uint8_t device_address,
  uint8_t register_address,
  uint16_t off_time)
{
  wire_ptr_->beginTransmission(device_address);
  wire_ptr_->write(register_address);
  wire_ptr_->write(off_time);
  wire_ptr_->write(off_time >> BITS_PER_BYTE);
  wire_ptr_->endTransmission();
}

void PCA9685::setOutputsInverted(uint8_t device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.invrt = OUTPUTS_INVERTED;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsNotInverted(uint8_t device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.invrt = OUTPUTS_NOT_INVERTED;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsToTotemPole(uint8_t device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.outdrv = OUTPUTS_TOTEM_POLE;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsToOpenDrain(uint8_t device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.outdrv = OUTPUTS_OPEN_DRAIN;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsLowWhenDisabled(uint8_t device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.outne = OUTPUTS_LOW_WHEN_DISABLED;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsHighWhenDisabled(uint8_t device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.outne = OUTPUTS_HIGH_WHEN_DISABLED;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsHighImpedanceWhenDisabled(uint8_t device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.outne = OUTPUTS_HIGH_IMPEDANCE_WHEN_DISABLED;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}
