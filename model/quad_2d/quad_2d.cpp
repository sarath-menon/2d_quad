#include "quad_2d.h"
#include <array>
#include <valarray>

// void Quad2D::motor_dynamics() {
//   actual_thrust_dot = (actual_thrust - commanded_thrust) /
//   motor_time_constant;
// }

void Quad2D::motor_speed_to_thrust_map(float motor_commands[4]) {
  for (int i = 0; i < 4; i++) {
    // std::cout << "Motor command " << i + 1 << ": " << motor_commands[i];
    motor_thrusts[i] = motor_commands[i] * motor_commands[i] * k_f_;
  }
  //   std::cout << std::endl;
}

/// Dynamics of the 2D quadcopter
// void Quad2D::dynamics(float thrust_input, float torque_input) {

//   // Neglect motor dynamics for now
//   actual_thrust = thrust_input;
//   actual_torque = torque_input;

//   x_ddot = actual_thrust * sin(beta) - drag_coeff_ * x_dot;

//   if (z < 0) {
//     // To prevent freefall into the ground
//     z = 0;
//     z_dot = 0;
//     z_ddot = 0;
//   } else
//     z_ddot = actual_thrust * cos(beta) - g - drag_coeff_ * fabs(z_dot);

//   beta_ddot = actual_torque / inertia_2d_;
// }

void Quad2D::new_dynamics(float motor_commands[4]) {

  motor_speed_to_thrust_map(motor_commands);
  // std::cout << "Simulator f1:" << motor_thrusts[0]
  //           << "\tf2:" << motor_thrusts[1] << "\tf3:" << motor_thrusts[2]
  //           << "\tf4:" << motor_thrusts[3] << std::endl;

  // // To be added later
  // motor_dynamics();

  float commanded_thrust =
      motor_thrusts[0] + motor_thrusts[1] + motor_thrusts[2] + motor_thrusts[3];

  float commanded_torque = (motor_thrusts[1] - motor_thrusts[3]) * arm_length_;

  // std::cout << "Net thrust and torque computer by simulator:"
  //           << commanded_thrust << '\t' << commanded_torque << std::endl;

  // Motor dynamics not considered for thrust since position loop much slower
  actual_thrust = commanded_thrust;
  // Neglect motor dynamics for now
  actual_torque = commanded_torque;

  x_ddot = actual_thrust * sin(beta) - drag_coeff_ * x_dot;

  if (z < 0) {
    // To prevent freefall into the ground
    z = 0;
    z_dot = 0;
    z_ddot = 0;
  } else
    z_ddot = actual_thrust * cos(beta) - g - drag_coeff_ * z_dot;

  beta_ddot = actual_torque / inertia_2d_;

  std::cout << "Angular acceleration:" << beta_ddot << '\n';
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

  // Initialize random number generator
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> d{0, 0.000};

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

void Quad2D::inner_loop_tuning_euler_step(float dt) {

  // Rotation
  beta = beta + beta_dot * dt;
  beta_dot = beta_dot + beta_ddot * dt;

  // Motor
  actual_thrust = actual_thrust_dot * dt + actual_thrust;
}