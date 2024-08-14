#include "servo.h"

#define LOGTAG "servo"

#ifdef ESP32
#define INTERVAL pwmFrequency*2
#else
#define INTERVAL 50*
#endif  

using namespace ModFirmWare;

#ifdef ESP32
Servo::Servo(uint8_t controlPin, uint8_t pwmChannel, uint8_t resolution, u_int frequency, u_int minPulse, u_int maxPulse) : Component()
//****************************************************************************************
{
  initialize(controlPin, 0, false, minPulse, maxPulse);
  this->pwmChannel = pwmChannel;
  this->pwmFrequency = frequency;
  this->pwmResolution = resolution;
  speed = calcPulseWidth((maxPulse - minPulse) / 4);  //speed is initialized in half of the pulse width per second

}

Servo::Servo(uint8_t controlPin, uint8_t enablePin, uint8_t pwmChannel, uint8_t resolution, u_int frequency, u_int minPulse, u_int maxPulse)
    : Component()
//****************************************************************************************
{
  initialize(controlPin, enablePin, true, minPulse, maxPulse);
  this->pwmChannel = pwmChannel;
  this->pwmFrequency = frequency;
  this->pwmResolution = resolution;
  speed = calcPulseWidth((maxPulse - minPulse) / 8);  //speed is initialized in quarter of the pulse width per second
}
#else
Servo::Servo(uint8_t controlPin, u_int minPulse, u_int maxPulse)
//****************************************************************************************
{
  initialize(controlPin, 0, false, minPulse, maxPulse);
}

Servo::Servo(uint8_t controlPin, uint8_t enablePin, u_int minPulse, u_int maxPulse)
//****************************************************************************************
{
  initialize(controlPin, enablePin, true, minPulse, maxPulse);
}
#endif // ESP32

bool Servo::setup(Application *app)
//****************************************************************************************
{
  if (!Component::setup(app))
  {
    return false;
  }

#ifdef ESP32
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  // Attach the PWM channel to the specified GPIO pin
  ledcAttachPin(controlPin, pwmChannel);
  logger->debug(LOGTAG, "Attaching servo to pwm channel = %d via pin %d (f:%d, r:%d)",
                pwmChannel, controlPin, pwmFrequency, pwmResolution);
#else
  pinMode(controlPin, OUTPUT);
  logger->debug(LOGTAG, "Attaching servo to pwm enabled pin %d", controlPin);
#endif // ESP32

  if (enabling)
  {
    pinMode(enablePin, OUTPUT);
    logger->debug(LOGTAG, "Attaching servo on/off to pin %d", enablePin);
  }

  return true;
}

void Servo::loop()
//****************************************************************************************
{
  if (currentPulse == targetPulse)
  {
    return;
  }

  if (INTERVAL > (millis() - lastChange))
  {
     return;
  }

  int delta = (int)((targetPulse > currentPulse) ? 1 : -1)  * (int)(speed * INTERVAL) / 1000;
  u_int pwm = currentPulse + delta;

  logger->debug(LOGTAG, "Incrementing %i to target = %d from %d", delta, targetPulse, currentPulse);


  if (delta > 0)
  {
    pwm = (targetPulse < pwm) ? targetPulse : (delta + currentPulse);
  }  
  else
  {
    pwm = (targetPulse > pwm) ? targetPulse : (delta + currentPulse);
  }

  #ifdef ESP32
  ledcWrite(pwmChannel, pwm);
  #else
  analogWriteFrequency(50);
  analogWrite(controlPin, pwm);  
  #endif
  currentPulse = pwm;
  lastChange = millis();

}

void Servo::setMaxAngle(float maxAngle)
//****************************************************************************************
{
  this->maxAngle = ((360 >= maxAngle) && (0 < maxAngle)) ? (long)(maxAngle * 100) : this->maxAngle;
  logger->debug(LOGTAG, "changed maxAngle of servo to %.2f", maxAngle / 100.0);
}

void Servo::setMaxPosition(long maxPosition)
//****************************************************************************************
{
  this->maxPosition = (maxPosition > this->minPosition) ? maxPosition : this->maxPosition;
  logger->debug(LOGTAG, "changed maxPosition of servo to %d", maxPosition);
}

void Servo::setMinPosition(long minPosition)
//****************************************************************************************
{
  this->minPosition = (minPosition < this->maxPosition) ? minPosition : this->minPosition;
  logger->debug(LOGTAG, "changed minPosition of servo to %d", minPosition);
}

void Servo::moveToAngle(float angle)
//****************************************************************************************
{
  long a = (long)round(angle * 100);
  applyPulseWidth(map(a, 0, maxAngle, minPulse, maxPulse));
}

void Servo::moveToPosition(long position)
//****************************************************************************************
{
  applyPulseWidth(map(position, minPosition, maxPosition, minPulse, maxPulse));
}

void Servo::disable()
//****************************************************************************************
{
  digitalWrite(enablePin, LOW);
}

void Servo::enable()
//****************************************************************************************
{
  digitalWrite(enablePin, HIGH);
}


void Servo::initialize(uint8_t cp, uint8_t ep, bool en, u_int minpw, u_int maxpw)
//****************************************************************************************
{
  controlPin = cp;
  enablePin = ep;
  enabling = en;

  minPulse = minpw;
  maxPulse = maxpw;
  currentPulse = 0;
  targetPulse = 0;
  lastChange = 0;

  maxAngle = 18000;
  minPosition = 0;
  maxPosition = 100;

  speed = 1;

}

void Servo::applyPulseWidth(int pw)
//****************************************************************************************
{  
  enable();
  targetPulse = calcPulseWidth(pw);
}

long Servo::calcPulseWidth(int pw)
{
#ifdef ESP32  
  int maxDutyCycle = (1 << pwmResolution) - 1; // Maximum duty cycle value based on resolution
  long pwm = (pw * maxDutyCycle) / (1000000 / pwmFrequency);

  logger->debug(LOGTAG, "Applying pulse width of %d resulting in duty of %d -- maxduty = %d", pw, pwm, maxDutyCycle);
#else
  long pwm = (pw * 1023) / 20000;
#endif // ESP32

  return pwm;
}