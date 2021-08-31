#include "quad_2d.h"
#include <array>

void Quad2D::motor_dynamics() {
  actual_thrust_dot = (actual_thrust - commanded_thrust) / motor_time_constant;
}

void Quad2D::motor_speed_to_thrust_map() {
  ;
  for (int i = 0; i < 3; i++) {
    motor_thrusts[i] = motor_speeds[i] * k_f * k_f;
  }
}

void Quad2D::thrust_allocation() {
  commanded_thrust =
      motor_speeds[0] + motor_speeds[1] + motor_speeds[2] + motor_speeds[2];

  commanded_torque = motor_speeds[1] - motor_speeds[3];
}

/// Dynamics of the 2D quadcopter
void Quad2D::dynamics(float thrust_input, float torque_input) {

  // Neglect motor dynamics for now
  actual_thrust = thrust_input;

  x_ddot = actual_thrust * sin(beta) - drag_coeff_ * x_dot;

  if (z < 0) {
    // To prevent freefall into the ground
    z = 0;
    z_dot = 0;
    z_ddot = 0;
  } else
    z_ddot = actual_thrust * cos(beta) - g - drag_coeff_ * fabs(z_dot);

  beta_ddot = torque_input / inertia_2d_;
}

void Quad2D::new_dynamics(float motor_speed[4]) {

  motor_speed_to_thrust_map();
  motor_dynamics();

  // // Neglect motor dynamics for now
  // actual_thrust = thrust_input;

  // x_ddot = actual_thrust * sin(beta) - drag_coeff_ * x_dot;

  // if (z < 0) {
  //   // To prevent freefall into the ground
  //   z = 0;
  //   z_dot = 0;
  //   z_ddot = 0;
  // } else
  //   z_ddot = actual_thrust * cos(beta) - g - drag_coeff_ * fabs(z_dot);

  // beta_ddot = torque_input / inertia_2d_;
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

  // Motor
  actual_thrust = actual_thrust_dot * dt + actual_thrust;

  // std::cout << "z_after_euler_step: " << z << std::endl;
  // std::cout << "z_dot_after_euler_step: " << z_dot << std::endl;
}

void Quad2D::sensor_read() {

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> d{0, 0.001};

  x_mes_ = x + d(gen);

  x_dot_mes_ = x_dot;

  z_mes_ = z;
  z_dot_mes_ = z_dot + d(gen);

  beta_mes_ = beta + d(gen);
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
