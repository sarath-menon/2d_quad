// #include "pid.h"
#include "motor_mixing.h"
#include "pid.h"
#include "plot.h"
#include "quad_2d.h"
#include "set_values.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <math_helper.h>
// Fastdds Headers
#include "mocap_quadcopterPubSubTypes.h"
#include "mocap_quadcopterPublisher.h"
// Px4 math header
#include "matrix/math.hpp"

int main() {
  Quad2D quad;

  // Initialize visualizer
  MyApp app;

  // Set quadcopter parameters
  quad.set_initial_conditions("project/parameters/initial_conditions.yaml");
  float motor_commands[4] = {0, 0, 0, 0};

  // // Fastdds publisher and message initialization
  bool fastdds_flag = false;
  mocap_quadcopterPublisher pose_pub;

  if (pose_pub_flag) {
    fastdds_flag = pose_pub.init();
  }

  // Outer Loop: Position Control
  for (int i = 0; i < euler_steps; i++) {

    // Get system state
    quad.sensor_read();

    // Compute error
    float altitude_error = altitude_target - quad.z_mes();
    float vertical_error = vertical_target - quad.x_mes();

    // Compute control input
    float thrust_command =
        altitude_pid(altitude_error, k_p__z, k_i__z, k_d__z, dt);

    // Quadcopter Motors have a maximum and minimum speed limit
    thrust_command =
        limit(ff_thrust + thrust_command, quad.thrust_max(), quad.thrust_min());

    float angle_command =
        vertical_pid(vertical_error, k_p__x, k_i__x, k_d__x, dt) / 9.81;

    angle_command = limit(angle_command, quad.roll_max(), -quad.roll_max());

    // Ony for tuning inner angle loop
    angle_command = 20;

    // Inner Loop: Angle Control
    float angle_error = angle_command - quad.beta_mes();

    float torque_command = roll_pid(angle_error, k_p__b, k_i__b, k_d__b, dt);

    torque_command =
        limit(torque_command, quad.torque_max(), -quad.torque_max());

    // // Apply control input and compute the change
    // quad.dynamics(thrust_command, torque_command);

    // // Convert thrust, torque to motor speeds
    motor_mixing(motor_commands, thrust_command, torque_command, quad.k_f(),
                 quad.arm_length());

    // Dynamics function that accepts motor commands instead of thrusts
    quad.new_dynamics(motor_commands);

    // // quad.dynamics(ff_thrust, torque_command);
    // quad.euler_step(dt);

    // Ony for tuning inner angle loop
    quad.inner_loop_tuning_euler_step(dt);

    // Diplay the control input and error
    // std::cout << "Thrust command:" << thrust_command << std::endl;
    // std::cout << "Altitude error:" << altitude_error << std::endl;
    // std::cout << "Angle Command:" << angle_command << std::endl;
    // std::cout << "Angle error:" << angle_error << std::endl;
    // std::cout << "Torque Command:" << torque_command << std::endl;
    // std::cout << "Vertical error:" << vertical_error << std::endl;
    // std::cout << "Motor commands:" << motor_commands[0] << std::endl;

    // Set variables for plotting
    plot_var::z_plot[i] = quad.true_z();
    plot_var::x_plot[i] = quad.true_x();
    plot_var::thrust_plot[i] = thrust_command;
    plot_var::torque_plot[i] = torque_command;
    plot_var::beta_plot[i] = quad.true_beta() * (180 / M_PI);
    plot_var::t_plot[i] = i * dt;

    if (pose_pub_flag && fastdds_flag) {
      // Publish mocap msg
      mocap_quadcopter msg;

      matrix::Eulerf euler(0, quad.true_beta(), 0);
      matrix::Quatf q_nb(euler);
      // std::cout << "q_w" << q_nb(0);

      msg.index({(uint32_t)i + 1});
      msg.position({quad.true_x() * 100, 0, quad.true_z() * 100});
      // msg.orientation_quaternion({0, 0, 0, 1});
      msg.orientation_quaternion({q_nb(1), q_nb(2), q_nb(3), q_nb(0)});
      pose_pub.run(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::cout << std::endl;
  // Plot the results
  app.run();
}
