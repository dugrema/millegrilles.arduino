#include "AdafruitSensors.h"

bool MilleGrillesAdafruitSensors::begin() {
  return _bmp.begin();
}

