#ifndef MODFIRMWARE_SERVO_H
#define MODFIRMWARE_SERVO_H

#include <Arduino.h>
#include <modfw_component.h>

namespace ModFirmWare
{
  class Servo : public Component
  {
  public:
#ifdef ESP32
    Servo(uint8_t controlPin, uint8_t pwmChannel, uint8_t resolution, u_int frequency, u_int minPulse, u_int maxPulse);
    Servo(uint8_t controlPin, uint8_t enablePin, uint8_t pwmChannel, uint8_t resolution, u_int frequency, u_int minPulse, u_int maxPulse);
#else
    Servo(uint8_t controlPin, u_int minPulse, u_int maxPulse);
    Servo(uint8_t controlPin, uint8_t enablePin, u_int minPulse, u_int maxPulse);
#endif // ESP32

    bool setup(Application* app);
    void loop();

    void setMaxAngle(float maxAngle);
    float getMaxAngle() { return maxAngle / 100.0;}
    void setMaxPosition(long maxPosition);
    void setMinPosition(long minPosition);

    void moveToAngle(float angle);
    void moveToPosition(long position);

  protected:
    long minPosition;
    long maxPosition;

    long maxAngle;

  private:
    uint8_t controlPin;
    uint8_t enablePin;
    bool enabling;

    u_int minPulse;
    u_int maxPulse;

#ifdef ESP32
    uint8_t pwmChannel;
    uint8_t pwmResolution;
    u_int pwmFrequency;
#endif

    void initialize(uint8_t cp, uint8_t ep, bool en, u_int minpw, u_int maxpw);
    void applyPulseWidth(int pw);
  };
};

#endif // MODFIRMWARE_SERVO_H