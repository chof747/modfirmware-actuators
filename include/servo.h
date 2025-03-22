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
    void invert(bool inverted = true);

    void moveToAngle(float angle, bool parkAfter = false);
    void moveToPosition(long position, bool parkAfter = false);

    void enable();
    void disable();

  protected:
    long minPosition;
    long maxPosition;

    long maxAngle;
    bool inverted;

  private:
    uint8_t controlPin;
    uint8_t enablePin;
    bool enabling;
    bool parking;

    u_int minPulse;
    u_int maxPulse;

    u_int currentPulse;
    u_int targetPulse;
    u_int lastChange;
    u_int speed;

#ifdef ESP32
    uint8_t pwmChannel;
    uint8_t pwmResolution;
    u_int pwmFrequency;
#endif

    void initialize(uint8_t cp, uint8_t ep, bool en, u_int minpw, u_int maxpw);
    void applyPulseWidth(int pw);
    long calcPulseWidth(int pw);
  };
};

#endif // MODFIRMWARE_SERVO_H