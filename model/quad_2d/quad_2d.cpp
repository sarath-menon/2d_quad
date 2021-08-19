#include "quad_2d.h"

/// Dynamics of the 2D quadcopter
void Quad2D::dynamics() {
  x_dot = x_dot_mes();
  x_ddot = actual_thrust * sin(beta) - drag_coeff * x_dot;

  z_dot = z_dot_mes();
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
}

void Quad2D::sensor_read() {

  x_mes_ = x;
  x_dot_mes_ = x_dot;

  z_mes_ = z;
  z_dot_mes_ = z_dot;

  beta_mes_ = beta;
  beta_dot_mes_ = beta_dot;
};