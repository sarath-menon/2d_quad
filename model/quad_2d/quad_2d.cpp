#include "quad_2d.h"
#include <array>
#include <valarray>

void Quad2D::motor_speed_to_thrust_map(float motor_commands[4]) {
  for (int i = 0; i < 4; i++) {
    // std::cout << "Motor command " << i + 1 << ": " << motor_commands[i];
    commanded_motor_thrusts[i] = motor_commands[i] * motor_commands[i] * k_f_;
    // Motor Dynamics
    actual_motor_thrusts_dot[i] =
        (commanded_motor_thrusts[i] - actual_motor_thrusts[i]) /
        motor_time_constant;
  }
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

  // Motor dynamics not considered for thrust since position loop much slower
  float commanded_thrust =
      commanded_motor_thrusts[0] + commanded_motor_thrusts[1] +
      commanded_motor_thrusts[2] + commanded_motor_thrusts[3];

  // float actual_thrust = actual_motor_thrusts[0] + actual_motor_thrusts[1] +
  //                       actual_motor_thrusts[2] + actual_motor_thrusts[3];

  actual_thrust = commanded_thrust;

  // Useful for logging and comparison
  float commanded_torque =
      (commanded_motor_thrusts[1] - commanded_motor_thrusts[3]) * arm_length_;

  // Motor dynamics is considered for attitude control lopp since it's rate is
  // comparable to the motor dynamics
  actual_torque =
      (actual_motor_thrusts[1] - actual_motor_thrusts[3]) * arm_length_;

  std::cout << "Commanded thrust and torque:" << commanded_thrust << '\t'
            << commanded_torque << std::endl;

  std::cout << "Actual thrust and torque produced by motors:" << actual_thrust
            << '\t' << actual_torque << std::endl;

  // // Neglect motor dynamics for now
  // actual_torque = commanded_torque;

  x_ddot = actual_thrust * sin(beta) - drag_coeff_ * x_dot;

  if (z < 0) {
    // To prevent freefall into the ground
    z = 0;
    z_dot = 0;
    z_ddot = 0;
  } else
    z_ddot = actual_thrust * cos(beta) - g - drag_coeff_ * fabs(z_dot);

  beta_ddot = actual_torque / inertia_2d_;
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

  //  motor dynamics
  for (int i = 0; i < 4; i++) {
    actual_motor_thrusts[i] =
        actual_motor_thrusts[i] + actual_motor_thrusts_dot[i] * dt;
  }
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