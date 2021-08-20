// #include "pid.h"
#include "pid.h"
#include "plot.h"
#include "quad_2d.h"
#include <iostream>

int main() {
  Quad2D quad;

  // Initialize visualizer
  MyApp app;

  float altitude_target = 10;
  float euler_steps = 1000;

  // PID Gains
  float k_p = 20;
  float k_i = 0;
  float k_d = 0;

  for (int i = 0; i < euler_steps; i++) {
    // Get system state
    quad.sensor_read();

    // Compute control input
    float altitude_error = altitude_target - quad.z_mes();
    float thrust_input = pid(altitude_error, k_p, k_i, k_d);

    // Diplay the control input and error
    std::cout << "Thrust input:" << thrust_input << std::endl;
    std::cout << "Altitude error:" << altitude_error << std::endl;

    // Apply control input and compute the change
    quad.dynamics(thrust_input, 0);
    quad.euler_step();

    plot_var::z_plot[i] = quad.z_mes();
    plot_var::actuator_plot[i] = thrust_input;
    plot_var::t_plot[i] = i;

    std::cout << std::endl;
  }

  // Plot the results
  app.run();
}