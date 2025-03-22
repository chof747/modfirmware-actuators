#ifndef CALIBRATED_SCALE
#define CALIBRATED_SCALE

#include <map>

namespace ModFirmWare
{

  class CalibratedScale
  {
  private:
    std::map<int, int> calibrationPoints;

  public:
    int operator()(int input);
    void addCalibrationPoint(int input, int mappedOutput);
    void clearCalibration();
    
  };

};

#endif // CALIBRATED_SCALE