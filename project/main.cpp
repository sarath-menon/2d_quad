// #include "pid.h"
#include "pid.h"
#include "plot.h"
#include "quad_2d.h"
#include "set_values.h"
#include <iostream>
#include <math.h>
#include <math_helper.h>

int main() {
  Quad2D quad;

  // Initialize visualizer
  MyApp app;

  // Set quadcopter parameters
  quad.set_initial_conditions("project/parameters/initial_conditions.yaml");

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
    // angle_command = 20;

    // Inner Loop: Angle Control
    float angle_error = angle_command - quad.beta_mes();

    float torque_command = roll_pid(angle_error, k_p__b, k_i__b, k_d__b, dt);

    torque_command =
        limit(torque_command, quad.torque_max(), -quad.torque_max());

    // Apply control input and compute the change
    quad.dynamics(thrust_command, torque_command);
    // quad.dynamics(ff_thrust, torque_command);
    quad.euler_step(dt);

    // Diplay the control input and error
    // std::cout << "Thrust command:" << thrust_command << std::endl;
    // std::cout << "Altitude error:" << altitude_error << std::endl;
    // std::cout << "Angle Command:" << angle_command << std::endl;
    // std::cout << "Angle error:" << angle_error << std::endl;
    // std::cout << "Torque Command:" << torque_command << std::endl;
    // std::cout << "Vertical error:" << vertical_error << std::endl;

    // Set variables for plotting
    plot_var::z_plot[i] = quad.z_mes();
    plot_var::x_plot[i] = quad.x_mes();
    plot_var::thrust_plot[i] = thrust_command;
    plot_var::torque_plot[i] = torque_command;
    plot_var::beta_plot[i] = quad.beta_mes();
    plot_var::t_plot[i] = i * dt;

    std::cout << std::endl;
  }

  // Plot the results
  app.run();
}