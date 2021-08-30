#pragma once
#include <yaml-cpp/yaml.h>

// Controller Properties

YAML::Node controller_yaml_file =
    YAML::LoadFile("project/parameters/controller_parameters.yaml");

// Altitude PID Gains
const float k_p__z = controller_yaml_file["k_p__z"].as<float>();
const float k_i__z = controller_yaml_file["k_i__z"].as<float>();
const float k_d__z = controller_yaml_file["k_d__z"].as<float>();

// Translation PID Gains
const float k_p__x = controller_yaml_file["k_p__z"].as<float>();
const float k_i__x = controller_yaml_file["k_i__z"].as<float>();
const float k_d__x = controller_yaml_file["k_d__z"].as<float>();

// feedforward thrust = - g
const float ff_thrust = 9.81;

// Simulation Properties

YAML::Node sim_yaml_file =
    YAML::LoadFile("project/parameters/simulation_parameters.yaml");

const float altitude_target = sim_yaml_file["altitude_target"].as<float>();

// Euler integration timestep
const float dt = sim_yaml_file["dt"].as<float>();
const float euler_steps = sim_yaml_file["euler_steps"].as<float>();