// #include "pid.h"
#include "pid.h"
#include "plot.h"
#include "quad_2d.h"
#include <iostream>

int main() {
  Quad2D quad;

  // Initialize visualizer
  MyApp app;

  float altitude_target = 5;
  float euler_steps = 100;

  for (int i = 0; i < euler_steps; i++) {
    // Get system state
    quad.sensor_read();

    // Compute control input
    float altitude_error = altitude_target - quad.z_mes();
    float thrust_input = pid(altitude_error, 20, 0, 0);

    // Diplay the control input and error
    std::cout << "Thrust input:" << thrust_input << std::endl;
    std::cout << "Altitude error:" << altitude_error << std::endl;

    // Apply control input and compute the change
    quad.dynamics(thrust_input, 0);
    quad.euler_step();

    plot_var::xs[i] = i;
    plot_var::ys[i] = quad.z_mes();

    std::cout << std::endl;
  }

  // Plot the results
  app.run();
}