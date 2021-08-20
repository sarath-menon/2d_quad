#include "quad_2d.h"
#include <iostream>

/// Dynamics of the 2D quadcopter
void Quad2D::dynamics(float thrust_input, float torque_input) {

  // Neglect motor dynamics for now
  actual_thrust = thrust_input;

  // x_dot = x_dot_mes();
  x_ddot = actual_thrust * sin(beta) - drag_coeff * x_dot;

  // z_dot = z_dot_mes();
  z_ddot = actual_thrust * cos(beta) - g - drag_coeff * z_dot;
}

void Quad2D::euler_step() {
  // Declare dt for now

  // Translation
  x = x + x_dot * dt;
  x_dot = x_dot + x_ddot * dt;

  z = z + z_dot * dt;
  z_dot = z_dot + z_ddot * dt;

  // Rotation
  beta = beta + beta_dot * dt;
  beta_dot = beta_dot + beta_ddot * dt;

  std::cout << "z_after_euler_step: " << z << std::endl;
  std::cout << "z_dot_after_euler_step: " << z_dot << std::endl;
}

void Quad2D::sensor_read() {

  x_mes_ = x;
  x_dot_mes_ = x_dot;

  z_mes_ = z;
  z_dot_mes_ = z_dot;

  beta_mes_ = beta;
  beta_dot_mes_ = beta_dot;

  std::cout << "z_mes_after_sensor_read: " << z << std::endl;
  std::cout << "z_dot_mes_after_sensor_read: " << z_dot << std::endl;
};