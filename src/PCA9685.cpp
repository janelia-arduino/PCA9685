// ----------------------------------------------------------------------------
// PCA9685.cpp
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#include "PCA9685.h"


void PCA9685::setup(TwoWire & wire,
  uint8_t slave_address)
{
  wire_ptr_ = &wire;
  slave_address_ = slave_address;
  output_enable_pin_ = NO_OUTPUT_ENABLE_PIN;

  wire_ptr_->begin();
  resetAllBusDevices();
}

void PCA9685::setup(uint8_t address)
{
  setup(Wire,address);
}

void PCA9685::setOutputEnablePin(size_t pin)
{
  pinMode(pin,OUTPUT);
  output_enable_pin_ = pin;
  disableOutputs();
}

void PCA9685::enableOutputs()
{
  if (output_enable_pin_ != NO_OUTPUT_ENABLE_PIN)
  {
    digitalWrite(output_enable_pin_,LOW);
  }
}

void PCA9685::disableOutputs()
{
  if (output_enable_pin_ != NO_OUTPUT_ENABLE_PIN)
  {
    digitalWrite(output_enable_pin_,HIGH);
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

void PCA9685::setAllChannelsFrequency(uint16_t frequency)
{
  uint16_t period_us = MICROSECONDS_PER_SECOND / frequency;
  period_us = constrain(period_us,PWM_PERIOD_MIN_US,PWM_PERIOD_MAX_US);
  uint8_t prescale = map(period_us,PWM_PERIOD_MIN_US,PWM_PERIOD_MAX_US,PRE_SCALE_MIN,PRE_SCALE_MAX);
  setPrescale(prescale);
}

uint16_t PCA9685::getTimeMin()
{
  return TIME_MIN;
}

uint16_t PCA9685::getTimeMax()
{
  return TIME_MAX;
}

void PCA9685::setChannelOnAndOffTimes(uint8_t channel,
  uint16_t on_time,
  uint16_t off_time)
{
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * channel;
  setOnAndOffTimes(register_address,on_time,off_time);
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

void PCA9685::writeByte(uint8_t register_address,
  uint8_t data)
{
  wire_ptr_->beginTransmission(slave_address_);
  wire_ptr_->write(register_address);
  wire_ptr_->write(data);
  wire_ptr_->endTransmission();
}

uint8_t PCA9685::readByte(uint8_t register_address)
{
  wire_ptr_->beginTransmission(slave_address_);
  wire_ptr_->write(register_address);
  wire_ptr_->endTransmission();

  wire_ptr_->requestFrom(slave_address_,READ_BYTE_COUNT);
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

void PCA9685::resetAllBusDevices()
{
  wire_ptr_->beginTransmission(GENERAL_CALL_SLAVE_ADDRESS);
  wire_ptr_->write(SWRST);
  wire_ptr_->endTransmission();
  delay(10);
  wake();
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

void PCA9685::setPrescale(uint8_t prescale)
{
  sleep();
  writeByte(PRE_SCALE_REGISTER_ADDRESS,prescale);
  wake();
}

void PCA9685::setOnAndOffTimes(uint8_t register_address,
  uint16_t on_time,
  uint16_t off_time)
{
  wire_ptr_->beginTransmission(slave_address_);
  wire_ptr_->write(register_address);
  wire_ptr_->write(on_time);
  wire_ptr_->write(on_time >> BITS_PER_BYTE);
  wire_ptr_->write(off_time);
  wire_ptr_->write(off_time >> BITS_PER_BYTE);
  wire_ptr_->endTransmission();
}

void PCA9685::setOnTime(uint8_t register_address,
  uint16_t on_time)
{
  wire_ptr_->beginTransmission(slave_address_);
  wire_ptr_->write(register_address);
  wire_ptr_->write(on_time);
  wire_ptr_->write(on_time >> BITS_PER_BYTE);
  wire_ptr_->endTransmission();
}

void PCA9685::setOffTime(uint8_t register_address,
  uint16_t off_time)
{
  wire_ptr_->beginTransmission(slave_address_);
  wire_ptr_->write(register_address);
  wire_ptr_->write(off_time);
  wire_ptr_->write(off_time >> BITS_PER_BYTE);
  wire_ptr_->endTransmission();
}
