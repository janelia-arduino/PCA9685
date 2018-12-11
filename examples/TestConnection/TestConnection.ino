#include <Arduino.h>
#include <Wire.h>
#include <Streaming.h>
#include <PCA9685.h>


const long BAUD = 115200;
const int LOOP_DELAY = 4000;
const int SLAVE_ADDRESS = 0x40;
const int PWM_FREQUENCY = 50;

PCA9685 pca9685;

void setup()
{
  Serial.begin(BAUD);

  pca9685.setup(Wire,SLAVE_ADDRESS);
  pca9685.setPwmFrequency(PWM_FREQUENCY);
}

void loop()
{
  delay(LOOP_DELAY);
  Serial << "min_pwm_frequency = " << pca9685.getMinPwmFrequency() << "\n";
  Serial << "max_pwm_frequency = " << pca9685.getMaxPwmFrequency() << "\n";
  // pca9685.setChannelOnAndOffTimes(0,0,2000);
  // delay(LOOP_DELAY);
  // pca9685.setPwmFrequency(1000);
  // delay(LOOP_DELAY);
  // pca9685.setPwmFrequency(25);
  // Serial << "...\n";
}
