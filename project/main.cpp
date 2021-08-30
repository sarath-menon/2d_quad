// #include "pid.h"
#include "pid.h"
#include "plot.h"
#include "quad_2d.h"
#include "set_values.h"
#include <iostream>
#include <math.h>

int main() {
  Quad2D quad;

  // Initialize visualizer
  MyApp app;

  // Set quadcopter parameters
  quad.set_parameters();

  for (int i = 0; i < euler_steps; i++) {

    // Get system state
    quad.sensor_read();

    // Compute error
    float altitude_error = altitude_target - quad.z_mes();
    // Compute control input
    float pid_output = pid(altitude_error, k_p__z, k_i__z, k_d__z, dt);
    // Motors have a maximum speed limit
    thrust_command = std::fmin(ff_thrust + pid_output, quad.thrust_max());
    // Motors cant be rotated in reverse during flight
    thrust_command = std::fmax(thrust_command, quad.thrust_min());

    // Diplay the control input and error
    std::cout << "Thrust command:" << thrust_command << std::endl;
    std::cout << "Altitude error:" << altitude_error << std::endl;

    // Apply control input and compute the change
    quad.dynamics(thrust_command, 0);
    quad.euler_step(dt);

    // Set variables for plotting
    plot_var::z_plot[i] = quad.z_mes();
    // plot_var::x_plot[i] = quad.x_mes();
    plot_var::actuator_plot[i] = thrust_command;
    plot_var::altitude_error_plot[i] = altitude_error;
    plot_var::t_plot[i] = i * dt;

    std::cout << std::endl;
  }

  // Plot the results
  app.run();
}