// #include "pid.h"
#include "pid.h"
#include "plot.h"
#include "quad_2d.h"
#include <iostream>
#include <math.h>

int main() {
  Quad2D quad;

  // Initialize visualizer
  MyApp app;

  float altitude_target = 5;

  // PID Gains
  float k_p = 4;
  float k_i = 0;
  float k_d = 0;
  float thrust_input = 0.0;

  // Euler integration timestep
  constexpr static float dt = 0.01;
  constexpr static float euler_steps = 500;

  for (int i = 0; i < euler_steps; i++) {
    // Get system state
    quad.sensor_read();

    // Compute control input
    float altitude_error = altitude_target - quad.z_mes();
    thrust_input = fmin(pid(altitude_error, k_p, k_i, k_d), quad.thrust_max());

    // Diplay the control input and error
    std::cout << "Thrust input:" << thrust_input << std::endl;
    std::cout << "Altitude error:" << altitude_error << std::endl;

    // Apply control input and compute the change
    quad.dynamics(thrust_input, 0);
    quad.euler_step(dt);

    plot_var::z_plot[i] = quad.z_mes();
    plot_var::actuator_plot[i] = thrust_input;
    plot_var::error_plot[i] = altitude_error;
    plot_var::t_plot[i] = i * dt;

    std::cout << std::endl;
  }

  // Plot the results
  app.run();
}