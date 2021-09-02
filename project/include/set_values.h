#pragma once
#include <yaml-cpp/yaml.h>

///////////////////////////////////////////////////////////////////////////////////////////
// Controller Properties
///////////////////////////////////////////////////////////////////////////////////////////
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

// Angle PID Gains
const float k_p__b = controller_yaml_file["k_p__b"].as<float>();
const float k_i__b = controller_yaml_file["k_i__b"].as<float>();
const float k_d__b = controller_yaml_file["k_d__b"].as<float>();

// feedforward thrust = - g
const float ff_thrust = 9.81;

///////////////////////////////////////////////////////////////////////////////////////////
// Simulation Properties
///////////////////////////////////////////////////////////////////////////////////////////

YAML::Node sim_yaml_file =
    YAML::LoadFile("project/parameters/simulation_parameters.yaml");

const float altitude_target = sim_yaml_file["altitude_target"].as<float>();
const float vertical_target = sim_yaml_file["vertical_target"].as<float>();

// Euler integration timestep
const float dt = sim_yaml_file["dt"].as<float>();
const float euler_steps = sim_yaml_file["euler_steps"].as<float>();

// Fastdds publisher activate or not
const bool pose_pub_flag = sim_yaml_file["pose_pub"].as<bool>();

///////////////////////////////////////////////////////////////////////////////////////////
// Plot Properties
///////////////////////////////////////////////////////////////////////////////////////////

YAML::Node plot_yaml_file =
    YAML::LoadFile("project/parameters/plot_parameters.yaml");

const bool altitude_plot_flag = plot_yaml_file["altitude_plot"].as<bool>();
const bool translation_plot_flag =
    plot_yaml_file["translation_plot"].as<bool>();
const bool thrust_plot_flag = plot_yaml_file["thrust_plot"].as<bool>();
const bool torque_plot_flag = plot_yaml_file["torque_plot"].as<bool>();
const bool roll_angle_flag = plot_yaml_file["roll_angle_plot"].as<bool>();