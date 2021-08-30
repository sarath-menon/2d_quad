#pragma once
#include <yaml-cpp/yaml.h>

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
