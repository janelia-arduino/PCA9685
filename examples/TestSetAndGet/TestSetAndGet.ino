#include <Arduino.h>
#include <PCA9685.h>
#include <Streaming.h>

#include "Constants.h"


PCA9685 pca9685;

uint16_t time_min;
uint16_t time_max;

uint8_t channel;

uint16_t on_time_set;
uint16_t off_time_set;
uint16_t on_time_get;
uint16_t off_time_get;

uint16_t pulse_width_set;
uint16_t phase_shift_set;
uint16_t pulse_width_get;
uint16_t phase_shift_get;

double duty_cycle_set;
double percent_delay_set;
double duty_cycle_get;
double percent_delay_get;

void setup()
{
  Serial.begin(constants::baud);

  pca9685.setupSingleDevice(Wire,constants::device_address);

  pca9685.setupOutputEnablePin(constants::output_enable_pin);
  pca9685.enableOutputs(constants::output_enable_pin);

  pca9685.setOutputsNotInverted();

  pca9685.setToServoFrequency();

  time_min = pca9685.getTimeMin();
  time_max = pca9685.getTimeMax();

  channel = 0;
  on_time_set = time_min;
  off_time_set = time_max;
}

void checkMatch(const char * s, bool match)
{
  if (match)
  {
    Serial << s << " get matches set.\n";
  }
  else
  {
    Serial << s << " get does not match set!\n";
  }
}

void loop()
{
  if (on_time_set > off_time_set)
  {
    on_time_set = time_min;
    off_time_set = time_max;
  }
  for (uint8_t channel=0; channel < pca9685.getChannelCount(); ++channel)
  {
    Serial << "frequency: " << pca9685.getFrequency() << "\n";
    Serial << "channel: " << channel << ", on_time_set: " << on_time_set << ", off_time_set: " << off_time_set << "\n";
    pca9685.setChannelOnAndOffTime(channel,on_time_set,off_time_set);
    pca9685.getChannelOnAndOffTime(channel,on_time_get,off_time_get);
    checkMatch("ChannelOnAndOffTime",((on_time_set == on_time_get) && (off_time_set == off_time_get)));

    pca9685.setChannelOnTime(channel,on_time_set);
    pca9685.getChannelOnTime(channel,on_time_get);
    checkMatch("ChannelOnTime",(on_time_set == on_time_get));

    pca9685.setChannelOffTime(channel,off_time_set);
    pca9685.getChannelOffTime(channel,off_time_get);
    checkMatch("ChannelOffTime",(off_time_set == off_time_get));

    pulse_width_set = off_time_set - on_time_set;
    phase_shift_set = on_time_set;
    Serial << "channel: " << channel << ", pulse_width_set: " << pulse_width_set << ", phase_shift_set: " << phase_shift_set << "\n";
    pca9685.setChannelPulseWidth(channel,pulse_width_set,phase_shift_set);
    pca9685.getChannelPulseWidth(channel,pulse_width_get,phase_shift_get);
    checkMatch("ChannelPulseWidth",((pulse_width_set == pulse_width_get) && (phase_shift_set == phase_shift_get)));

    duty_cycle_set = (pulse_width_set * pca9685.getDutyCycleMax()) / pca9685.getTimeMax();
    percent_delay_set = (phase_shift_set * pca9685.getPercentDelayMax()) / pca9685.getTimeMax();
    Serial << "channel: " << channel << ", duty_cycle_set: " << duty_cycle_set << ", percent_delay_set: " << percent_delay_set << "\n";
    pca9685.setChannelDutyCycle(channel,duty_cycle_set,percent_delay_set);
    pca9685.getChannelDutyCycle(channel,duty_cycle_get,percent_delay_get);
    checkMatch("ChannelDutyCycle",((abs(duty_cycle_set - duty_cycle_get) < constants::epsilon) && (abs(percent_delay_set - percent_delay_get) < constants::epsilon)));

    Serial << "\n";
    delay(constants::loop_delay);
  }
  on_time_set += constants::time_increment;
  off_time_set -= constants::time_increment;
}
