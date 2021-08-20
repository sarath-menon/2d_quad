#pragma once
#include <math.h>

/// Represents the swinging arm
class Quad2D {

private:
  // Position and Orientation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Position
  float x = 0; // m
  float z = 2; // m

  // Velocities
  float x_dot = 0;   // m/s
  float z_dot = 5.5; // m/s

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
  constexpr static float arm_length = 0.1; // [m]
  constexpr static float inertia_2d = 0.1; // [kg m^2]

  /// Thrust generated by the motor propellor pair
  float actual_thrust = 0;
  float commanded_thrust = 0;

  /// Torque generated by the motor propellor pair
  float actual_torque = 0;
  float commanded_torque = 0;

  // Variables for dynamics function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Gravitational constant
  constexpr static float g = 9.81;

  // Air drag coefficient
  constexpr static float drag_coeff = 0.2;

  // Maximum thrust can be produced by the motors
  constexpr static float thrust_max_ = 25;

  // Measured states ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  float x_mes_ = 0;
  float x_dot_mes_ = 0;

  float z_mes_ = 0;
  float z_dot_mes_ = 0;

  float beta_mes_ = 0;
  float beta_dot_mes_ = 0;

public:
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

  /// Setter function
  // void set_x_mes(float x_mes) { x_mes_ = x_mes; }
  // /// Setter function
  // void set_x_dot_mes(float x_mes) { x_mes_ = x_mes; }
  // /// Setter function
  // void set_z_mes(float z_mes) { z_mes_ = z_mes; }
  // /// Setter function
  // void set_z_dot_mes(float z_mes) { z_mes_ = z_mes; }
  // /// Setter function
  // void set_beta_mes(float beta_mes) { beta_mes_ = beta_mes; }
  // /// Setter function
  // void set_beta_dot_mes(float beta_mes) { beta_mes_ = beta_mes; }
};