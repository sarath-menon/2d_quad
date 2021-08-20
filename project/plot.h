#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>

using namespace mahi::gui;
using namespace mahi::util;

namespace plot_var {
const int euler_timesteps = 1000;

float z_plot[euler_timesteps], actuator_plot[euler_timesteps],
    t_plot[euler_timesteps];

// Plot axes limits
const int x_min = 0;
const int x_max = euler_timesteps;
const int y_min = 0;
const int y_max = 20;

} // namespace plot_var

// Inherit from Application
class MyApp : public Application {
public:
  // 640x480 px window
  MyApp() : Application(640, 480, "My App") {}
  // Override update (called once per frame)

  void update() override {
    // App logic and/or ImGui code goes here
    ImGui::Begin("Example");

    static float alpha = 0.25f;
    ImGui::DragFloat("Alpha", &alpha, 0.01f, 0, 1);
    ImPlot::PushColormap(ImPlotColormap_Pastel);
    ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, plot_var::y_min,
                              plot_var::y_max);

    if (ImPlot::BeginPlot("Altitude vs Time", "X-Axis", "Y-Axis")) {
      ImPlot::PlotLine("Uncertain Data", plot_var::t_plot, plot_var::z_plot,
                       plot_var::euler_timesteps);
      ImPlot::EndPlot();
    }

    ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, -500, 500);
    if (ImPlot::BeginPlot("Thrust input vs Time", "X-Axis", "Y-Axis")) {
      ImPlot::PlotLine("Uncertain Data", plot_var::t_plot,
                       plot_var::actuator_plot, plot_var::euler_timesteps);
      ImPlot::EndPlot();
    }

    ImGui::End();
  }
};