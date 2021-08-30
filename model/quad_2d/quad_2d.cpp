#include "quad_2d.h"

/// Dynamics of the 2D quadcopter
void Quad2D::dynamics(float thrust_input, float torque_input) {

  // Neglect motor dynamics for now
  actual_thrust = thrust_input;

  // x_dot = x_dot_mes();
  x_ddot = actual_thrust * sin(beta) - drag_coeff * x_dot;

  // z_dot = z_dot_mes();
  if (z < 0) {
    // To prevent freefall into the ground
    z = 0;
    z_dot = 0;
    z_ddot = 0;
  } else
    z_ddot = actual_thrust * cos(beta) - g - drag_coeff * fabs(z_dot);
}

void Quad2D::euler_step(float dt) {
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

  // std::cout << "z_mes_after_sensor_read: " << z << std::endl;
  // std::cout << "z_dot_mes_after_sensor_read: " << z_dot << std::endl;
};

void Quad2D::set_parameters(std::string parameter_path) {

  YAML::Node yaml_file = YAML::LoadFile("project/parameters.yaml");

  mass_ = yaml_file["mass"].as<float>(); // [kg]

  float altitude_target = 5;
  float thrust_command = 0.0;

  // Altitude PID Gains
  float k_p__z = 6.5;
  float k_i__z = 0;
  float k_d__z = 2;

  // Translation PID Gains
  float k_p__x = 6.5;
  float k_i__x = 0;
  float k_d__x = 2;

  // Euler integration timestep
  constexpr static float dt = 0.01;
  constexpr static float euler_steps = 1000;

  // feedforward thrust = - g
  float ff_thrust = 9.81;
}
