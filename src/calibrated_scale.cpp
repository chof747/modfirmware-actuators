#include "calibrated_scale.h"

using namespace ModFirmWare;

int CalibratedScale::operator()(int input)
//****************************************************************************************
{
  auto a = calibrationPoints.lower_bound(input);

  if (a == calibrationPoints.end())
  {
    // input > max_value return max_value
    return (--a)->second;
  }
  if (a->first == input)
  {
    // exact matches can be returned directly
    return a->second;
  }

  if (a == calibrationPoints.begin())
  {
    //input < min_value return min_value
    return a->second;
  }

  auto b = std::prev(a);
  int x0 = a->first, y0 = a->second;
  int x1 = b->first, y1 = b->second;

  // Linear interpolation formula
  return y0 + ((y1 - y0) * (input - x0)) / (x1 - x0);
}

void CalibratedScale::addCalibrationPoint(int input, int mappedOutput)
//****************************************************************************************
{
  calibrationPoints[input] = mappedOutput;
}

void CalibratedScale::clearCalibration()
//****************************************************************************************
{
  calibrationPoints.clear();
}
