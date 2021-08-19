#pragma once
#include <math.h>

/// Represents the swinging arm
class Quad2D {

protected:
  // Position and Orientation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Position along z axis
  float z = 0;
  float z_dot = 0;
  float z_ddot = 0;

  // Position along x axis
  float x = 0;
  float x_dot = 0;
  float x_ddot = 0;

  // Orientatiom
  float beta = 0;
  float beta_dot = 0;
  float beta_ddot = 0;

  // Geometrical properties

  constexpr static float arm_length = 0.1; // [m]
  constexpr static float inertia_2d = 0.1; // [kg m^2]

  // Variables for dynamics function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Gravitational constant
  float g = 9.81;

  // Air drag coefficient
  float drag_coeff = 0.2;

  /// Thrust generated by the motor propellor pair
  float actual_thrust = 0;
  float commanded_thrust = 0;

  /// Torque generated by the motor propellor pair
  float actual_torque = 0;
  float commanded_torque = 0;

  // Measured states ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  float x_mes = 0;
  float x_dot_mes = 0;

  float z_mes = 0;
  float z_dot_mes = 0;

  float beta_mes = 0;
  float beta_dot_mes = 0;

public:
  /// Dynamics of the 2D quadcopter
  void Dynamics();

  // Numerical integration
  void euler_step();

  void add_sensor_noise(){};
};