#pragma once
#include <iostream>
#include <math.h>
#include <string.h>
#include <yaml-cpp/yaml.h>

/// Represents the swinging arm
class Quad2D {

private:
  // Load YAML file containing quad properties
  YAML::Node yaml_file = YAML::LoadFile("model/parameters.yaml");

  // Position and Orientation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Position
  float x = 0; // m
  float z = 0; // m

  // Velocities
  float x_dot = 0; // m/s
  float z_dot = 0; // m/s

  // Accelerations
  float x_ddot = 0; // m/s^2
  float z_ddot = 0; // m/s^2

  // Orientation
  float beta = 0;

  // Angular velocity
  float beta_dot = 0;

  // Angular acceleration
  float beta_ddot = 0;

  // Geometrical properties
  float arm_length_ = yaml_file["arm_length"].as<float>(); // [m]
  float inertia_2d_ = yaml_file["inertia_2d"].as<float>(); // [kg m^2]

  /// Thrust generated by the motor propellor pair
  float actual_thrust = 0;
  float commanded_thrust = 0;

  /// Torque generated by the motor propellor pair
  float actual_torque = 0;
  float commanded_torque = 0;

  // Variables for dynamics function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Mass of the quadcopter
  float mass_ = yaml_file["mass"].as<float>();

  // Gravitational constant
  constexpr static float g = 9.81;

  // Air drag coefficient
  float drag_coeff_ = yaml_file["drag_coeff"].as<float>();

  // Maximum thrust can be produced by the motors
  float thrust_max_ = yaml_file["thrust_max"].as<float>();

  // Maximum thrust can be produced by the motors
  float thrust_min_ = yaml_file["thrust_min"].as<float>();

  // Maximum thrust can be produced by the motors
  float roll_max_ = yaml_file["roll_max"].as<float>();

  // Maximum thrust can be produced by the motors
  const float torque_max_ = 2 * thrust_max_ * arm_length_;

  // Measured states ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  float x_mes_ = 0;
  float x_dot_mes_ = 0;

  float z_mes_ = 0;
  float z_dot_mes_ = 0;

  float beta_mes_ = 0;
  float beta_dot_mes_ = 0;

public:
  /// Set quadcopter parameters
  void set_initial_conditions(std::string path);

  /// Dynamics of the 2D quadcopter
  void dynamics(float thrust_input, float torque_input);

  // Numerical integration
  void euler_step(float dt);

  // Get sensor measurements by adding artifical noise to the sensors
  void sensor_read();

public:
  /// Getter function
  float x_mes() const { return x_mes_; }
  /// Getter function
  float x_dot_mes() const { return x_dot_mes_; }
  /// Getter function
  float z_mes() const { return z_mes_; }
  /// Getter function
  float z_dot_mes() const { return z_dot_mes_; }
  /// Getter function
  float beta_mes() const { return beta_mes_; }
  /// Getter function
  float beta_dot_mes() const { return beta_dot_mes_; }
  /// Getter function
  float thrust_max() const { return thrust_max_; }
  /// Getter function
  float thrust_min() const { return thrust_min_; }
  /// Getter function
  float torque_max() const { return torque_max_; }
  /// Getter function
  float roll_max() const { return roll_max_; }
  /// Getter function
  float mass() const { return mass_; }

  /// Setter function
  void set_x_mes(float x_mes) { x_mes_ = x_mes; }
  /// Setter function
  void set_x_dot_mes(float x_mes) { x_mes_ = x_mes; }
  /// Setter function
  void set_z_mes(float z_mes) { z_mes_ = z_mes; }
  /// Setter function
  void set_z_dot_mes(float z_mes) { z_mes_ = z_mes; }
  /// Setter function
  void set_beta_mes(float beta_mes) { beta_mes_ = beta_mes; }
  /// Setter function
  void set_beta_dot_mes(float beta_mes) { beta_mes_ = beta_mes; }
  /// Setter function
  void set_mass(float mass) { mass_ = mass; }
};