#include "quad_2d.h"

/// Dynamics of the 2D quadcopter
void Quad2D::dynamics(float thrust_input, float torque_input) {

  // Neglect motor dynamics for now
  actual_thrust = thrust_input;

  // x_dot = x_dot_mes();
  x_ddot = actual_thrust * sin(beta) - drag_coeff_ * x_dot;

  // z_dot = z_dot_mes();
  if (z < 0) {
    // To prevent freefall into the ground
    z = 0;
    z_dot = 0;
    z_ddot = 0;
  } else
    z_ddot = actual_thrust * cos(beta) - g - drag_coeff_ * fabs(z_dot);
}

void Quad2D::euler_step(float dt) {

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

void Quad2D::set_initial_conditions(std::string path) {

  YAML::Node yaml_file = YAML::LoadFile(path);

  x = yaml_file["x"].as<float>();         // [m]
  z = yaml_file["z"].as<float>();         // [m]
  x_dot = yaml_file["x_dot"].as<float>(); // [m/s]
  z_dot = yaml_file["z_dot"].as<float>(); // [m/s]
}
