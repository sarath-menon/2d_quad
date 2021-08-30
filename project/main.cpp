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

    // Inner Loop: Angle Control
    float angle_command =
        -angle_pid(vertical_error, k_p__b, k_i__b, k_d__b, dt) / 9.81;

    float angle_error = angle_command - quad.beta_mes();

    float torque_command =
        vertical_pid(altitude_error, k_p__z, k_i__z, k_d__z, dt);

    // Apply control input and compute the change
    quad.dynamics(thrust_command, 0);
    quad.euler_step(dt);

    // Diplay the control input and error
    std::cout << "Thrust command:" << thrust_command << std::endl;
    std::cout << "Altitude error:" << altitude_error << std::endl;

    // Set variables for plotting
    plot_var::z_plot[i] = quad.z_mes();
    plot_var::x_plot[i] = quad.x_mes();
    plot_var::actuator_plot[i] = thrust_command;
    // plot_var::altitude_error_plot[i] = altitude_error;
    plot_var::t_plot[i] = i * dt;

    std::cout << std::endl;
  }

  // Plot the results
  app.run();
}