#pragma once
#include <math.h>

float limit(float val, const float max, const float min) {
  val = std::fmax(val, min);
  val = std::fmin(val, max);
  return val;
}
