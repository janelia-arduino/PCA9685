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

#include <Streaming.h>


class PCA9685
{
public:
  void setup(TwoWire & wire,
    uint8_t slave_address);
  void setup(uint8_t slave_address=0x40);

  uint16_t getPwmFrequencyMin();
  uint16_t getPwmFrequencyMax();
  void setPwmFrequency(uint16_t frequency);

  void sleep();
  void wake();

  uint16_t getTimeMin();
  uint16_t getTimeMax();
  void setChannelOnAndOffTimes(uint8_t channel,
    uint16_t on_time,
    uint16_t off_time);
  void setChannelOnTime(uint8_t channel,
    uint16_t on_time);
  void setChannelOffTime(uint8_t channel,
    uint16_t off_time);

private:
  TwoWire * wire_ptr_;
  uint8_t slave_address_;

  const static uint8_t GENERAL_CALL_SLAVE_ADDRESS = 0x00;
  const static uint8_t SWRST = 0b110;

  const static uint8_t READ_BYTE_COUNT = 1;

  void writeByte(uint8_t register_address,
    uint8_t data);
  uint8_t readByte(uint8_t register_address);

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
      uint8_t space : 3;
      uint8_t invrt : 1;
      uint8_t och : 1;
      uint8_t outdrv : 1;
      uint8_t outne : 2;
    } fields;
    uint8_t data;
  };
  Mode2Register readMode2Register();

  void resetAllBusDevices();
  void setPrescale(uint8_t prescale);

  const static uint8_t SUBADR1_REGISTER_ADDRESS = 0x02;
  const static uint8_t SUBADR2_REGISTER_ADDRESS = 0x03;
  const static uint8_t SUBADR3_REGISTER_ADDRESS = 0x04;
  const static uint8_t ALLCALLADR_REGISTER_ADDRESS = 0x05;

  const static uint8_t LED0_ON_L_REGISTER_ADDRESS = 0x06;
  const static uint8_t LED_REGISTERS_SIZE = 4;
  const static uint8_t BITS_PER_BYTE = 8;

  const static uint8_t ALL_LED_ON_L_REGISTER_ADDRESS = 0xFA;
  const static uint8_t ALL_LED_ON_H_REGISTER_ADDRESS = 0xFB;
  const static uint8_t ALL_LED_OFF_L_REGISTER_ADDRESS = 0xFC;
  const static uint8_t ALL_LED_OFF_H_REGISTER_ADDRESS = 0xFD;

  const static uint8_t PRE_SCALE_REGISTER_ADDRESS = 0xFE;
  const static uint8_t PRE_SCALE_MIN = 0x03;
  const static uint8_t PRE_SCALE_MAX = 0xFF;
  // Measured, should be 1E6/1526=655, variation in chips
  const static uint16_t PWM_PERIOD_MIN_US = 617;
  // Measured, should be 1E6/24=41666, variation in chips
  const static uint16_t PWM_PERIOD_MAX_US = 39525;
  const static uint32_t MICROSECONDS_PER_SECOND = 1000000;

  const static uint16_t TIME_MIN = 0;
  const static uint16_t TIME_MAX = 4095;

  const static uint8_t TEST_MODE_REGISTER_ADDRESS = 0xFF;
};

#endif
