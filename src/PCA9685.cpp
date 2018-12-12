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
  channel_count_ = 0;
  for (uint8_t device_index=0; device_index<DEVICE_COUNT_MAX; ++device_index)
  {
    output_enable_pins_[DEVICE_INDEX] = NO_OUTPUT_ENABLE_PIN;
  }
}

void PCA9685::setWire(TwoWire & wire)
{
  wire_ptr_ = &wire;
  wire_ptr_->begin();
}

uint8_t PCA9685::addDevice(uint8_t device_address)
{
  uint8_t device_index = device_count_;
  device_address_[device_index] = device_address;
  ++device_count_;
  channel_count_ += CHANNELS_PER_DEVICE;
  return device_index;
}

void PCA9685::resetAllDevices()
{
  wire_ptr_->beginTransmission(GENERAL_CALL_DEVICE_ADDRESS);
  wire_ptr_->write(SWRST);
  wire_ptr_->endTransmission();
  delay(10);
  wakeAll();
}

void PCA9685::setOutputEnablePin(uint8_t device_index,
  size_t pin)
{
  if (device_index < device_count_)
  {
    pinMode(pin,OUTPUT);
    output_enable_pins_[device_index] = pin;
    disableOutputs(device_index);
  }
}

void PCA9685::setAllOutputEnablePins(size_t pin)
{
  pinMode(pin,OUTPUT);
  for (uint8_t device_index=0; device_index<DEVICE_COUNT_MAX; ++device_index)
  {
    output_enable_pins_[device_index] = pin;
    disableOutputs(device_index);
  }
}

void PCA9685::enableOutputs(uint8_t device_index)
{
  if (device_index < device_count_)
  {
    if (output_enable_pins_[device_index] != NO_OUTPUT_ENABLE_PIN)
    {
      digitalWrite(output_enable_pin_,LOW);
    }
  }
}

void PCA9685::enableAllOutputs()
{
  for (uint8_t device_index=0; device_index<DEVICE_COUNT_MAX; ++device_index)
  {
    enableOutputs(device_index);
  }
}

void PCA9685::disableOutputs(uint8_t device_index)
{
  if (device_index < device_count_)
  {
    if (output_enable_pin_ != NO_OUTPUT_ENABLE_PIN)
    {
      digitalWrite(output_enable_pin_,HIGH);
    }
  }
}

void PCA9685::disableAllOutputs()
{
  for (uint8_t device_index=0; device_index<DEVICE_COUNT_MAX; ++device_index)
  {
    disableOutputs(device_index);
  }
}

uint16_t PCA9685::getFrequencyMin()
{
  return MICROSECONDS_PER_SECOND / PWM_PERIOD_MAX_US;
}

uint16_t PCA9685::getFrequencyMax()
{
  return MICROSECONDS_PER_SECOND / PWM_PERIOD_MIN_US;
}

void PCA9685::setFrequency(uint8_t device_index,
  uint16_t frequency)
{
  if (device_index < device_count_)
  {
    uint8_t prescale = frequencyToPrescale(frequency);
    setPrescale(device_index,prescale);
  }
}

void PCA9685::setAllFrequencies(uint16_t frequency)
{
  uint8_t prescale = frequencyToPrescale(frequency);
  setAllPrescales(prescale);
}

uint16_t PCA9685::getTimeMin()
{
  return TIME_MIN;
}

uint16_t PCA9685::getTimeMax()
{
  return TIME_MAX;
}

void PCA9685::setOnAndOffTime(uint8_t channel,
  uint16_t on_time,
  uint16_t off_time)
{
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * channel;
  setRegisterOnAndOffTime(register_address,on_time,off_time);
}

void PCA9685::setAllChannelsOnAndOffTimes(uint16_t on_time,
  uint16_t off_time)
{
  uint8_t register_address = ALL_LED_ON_L_REGISTER_ADDRESS;
  setOnAndOffTimes(register_address,on_time,off_time);
}

void PCA9685::setChannelOnTime(uint8_t channel,
  uint16_t on_time)
{
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * channel;
  setOnTime(register_address,on_time);
}

void PCA9685::setAllChannelsOnTime(uint16_t on_time)
{
  uint8_t register_address = ALL_LED_ON_L_REGISTER_ADDRESS;
  setOnTime(register_address,on_time);
}

void PCA9685::setChannelOffTime(uint8_t channel,
  uint16_t off_time)
{
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * channel + LED_REGISTERS_SIZE / 2;
  setOffTime(register_address,off_time);
}

void PCA9685::setAllChannelsOffTime(uint16_t off_time)
{
  uint8_t register_address = ALL_LED_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE / 2;
  setOffTime(register_address,off_time);
}

void PCA9685::setOutputsInverted()
{
  Mode2Register mode2_register = readMode2Register();
  mode2_register.fields.invrt = OUTPUTS_INVERTED;
  writeByte(MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsNotInverted()
{
  Mode2Register mode2_register = readMode2Register();
  mode2_register.fields.invrt = OUTPUTS_NOT_INVERTED;
  writeByte(MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsToTotemPole()
{
  Mode2Register mode2_register = readMode2Register();
  mode2_register.fields.outdrv = OUTPUTS_TOTEM_POLE;
  writeByte(MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsToOpenDrain()
{
  Mode2Register mode2_register = readMode2Register();
  mode2_register.fields.outdrv = OUTPUTS_OPEN_DRAIN;
  writeByte(MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsLowWhenDisabled()
{
  Mode2Register mode2_register = readMode2Register();
  mode2_register.fields.outne = OUTPUTS_LOW_WHEN_DISABLED;
  writeByte(MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsHighWhenDisabled()
{
  Mode2Register mode2_register = readMode2Register();
  mode2_register.fields.outne = OUTPUTS_HIGH_WHEN_DISABLED;
  writeByte(MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsHighImpedanceWhenDisabled()
{
  Mode2Register mode2_register = readMode2Register();
  mode2_register.fields.outne = OUTPUTS_HIGH_IMPEDANCE_WHEN_DISABLED;
  writeByte(MODE2_REGISTER_ADDRESS,mode2_register.data);
}

// private

void PCA9685::writeByte(uint8_t device_address,
  uint8_t register_address,
  uint8_t data)
{
  wire_ptr_->beginTransmission(device_address);
  wire_ptr_->write(register_address);
  wire_ptr_->write(data);
  wire_ptr_->endTransmission();
}

uint8_t PCA9685::readByte(uint8_t device_address,
  uint8_t register_address)
{
  wire_ptr_->beginTransmission(device_address);
  wire_ptr_->write(register_address);
  wire_ptr_->endTransmission();

  wire_ptr_->requestFrom(device_address,READ_BYTE_COUNT);
  return wire_ptr_->read();
}

PCA9685::Mode1Register PCA9685::readMode1Register()
{
  Mode1Register mode1_register;
  mode1_register.data = readByte(MODE1_REGISTER_ADDRESS);
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

PCA9685::Mode2Register PCA9685::readMode2Register()
{
  Mode2Register mode2_register;
  mode2_register.data = readByte(MODE2_REGISTER_ADDRESS);
  // Serial << "outne: " << mode2_register.fields.outne << "\n";
  // Serial << "outdrv: " << mode2_register.fields.outdrv << "\n";
  // Serial << "och: " << mode2_register.fields.och << "\n";
  // Serial << "invrt: " << mode2_register.fields.invrt << "\n";
  return mode2_register;
}

void PCA9685::sleep()
{
  Mode1Register mode1_register = readMode1Register();
  mode1_register.fields.sleep = SLEEP;
  writeByte(MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::wake()
{
  Mode1Register mode1_register = readMode1Register();
  mode1_register.fields.sleep = WAKE;
  mode1_register.fields.ai = AUTO_INCREMENT_ENABLED;
  writeByte(MODE1_REGISTER_ADDRESS,mode1_register.data);
  delay(1);
  if (mode1_register.fields.restart == RESTART_ENABLED)
  {
    mode1_register.fields.restart = RESTART_CLEAR;
    writeByte(MODE1_REGISTER_ADDRESS,mode1_register.data);
  }
}

uint8_t PCA9685::frequencyToPrescale(uint16_t frequency)
{
  uint16_t period_us = MICROSECONDS_PER_SECOND / frequency;
  period_us = constrain(period_us,PWM_PERIOD_MIN_US,PWM_PERIOD_MAX_US);
  uint8_t prescale = map(period_us,PWM_PERIOD_MIN_US,PWM_PERIOD_MAX_US,PRE_SCALE_MIN,PRE_SCALE_MAX);
  return prescale;
}

void PCA9685::setPrescale(uint8_t prescale)
{
  sleep();
  writeByte(PRE_SCALE_REGISTER_ADDRESS,prescale);
  wake();
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
