#pragma once
#include <math.h>

float pid(const float e, const float k_p, const float k_i, const float k_d,
          const float dt) {
  // e -> error
  // e_prev -> Previous error

  // Declare these variables for now
  static float e_i = 0;
  static float e_d = 0;
  static float e_prev = 0;

  // dt -> Current timestep - Previous timestep
  // current_time = get_system_time()
  // dt = current_time - prev_time;

  // e_i -> Error integral term
  e_i += e;

  // e_d -> Error derivative error
  e_d = (e - e_prev) / dt;

  float control_output = e * k_p + e_i * k_i + e_d * k_d;

  // Set current values as prev values
  // prev_time = current_time;
  e_prev = e;

  return control_output;
}
