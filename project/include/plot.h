#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>

using namespace mahi::gui;
using namespace mahi::util;

namespace plot_var {
const int euler_timesteps = 10;

// Variables to be plotted
float z_plot[euler_timesteps], x_plot[euler_timesteps],
    thrust_plot[euler_timesteps], torque_plot[euler_timesteps],
    beta_plot[euler_timesteps], t_plot[euler_timesteps];

// Plot axes limits
const int x_min = 0;
const int x_max = 5;
const int y_min = 0;
const int y_max = 20;

} // namespace plot_var

// Inherit from Application
class MyApp : public Application {
public:
  // 640x480 px window
  MyApp() : Application(1080, 720, "My App") {}
  // Override update (called once per frame)

  void update() override {
    // App logic and/or ImGui code goes here
    ImGui::Begin("Example");

    static float alpha = 0.25f;
    ImGui::DragFloat("Alpha", &alpha, 0.01f, 0, 1);
    ImPlot::PushColormap(ImPlotColormap_Pastel);
    ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, plot_var::y_min,
                              plot_var::y_max);
    // Altitude plot
    if (ImPlot::BeginPlot("Altitude vs Time", "time", "altitude",
                          ImVec2(-1, 200))) {
      ImPlot::PlotLine("altitude", plot_var::t_plot, plot_var::z_plot,
                       plot_var::euler_timesteps);
      ImPlot::EndPlot();
    }

    ImGui::DragFloat("Alpha", &alpha, 0.01f, 0, 1);
    ImPlot::PushColormap(ImPlotColormap_Pastel);
    ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, plot_var::y_min,
                              plot_var::y_max);
    // Translation plot
    if (ImPlot::BeginPlot("Vertical vs Time", "time", "x distance",
                          ImVec2(-1, 200))) {
      ImPlot::PlotLine("vertical distance", plot_var::t_plot, plot_var::x_plot,
                       plot_var::euler_timesteps);
      ImPlot::EndPlot();
    }

    // Thrust input plot
    ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, 0, 25);
    ImPlot::PushColormap(ImPlotColormap_Pastel);
    if (ImPlot::BeginPlot("Thrust input vs Time", "time", "thrust Input",
                          ImVec2(-1, 200))) {
      ImPlot::PlotLine("thrust input", plot_var::t_plot, plot_var::thrust_plot,
                       plot_var::euler_timesteps);
      ImPlot::EndPlot();
    }

    // Torque input plot
    ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, -40, 40);
    ImPlot::PushColormap(ImPlotColormap_Pastel);
    if (ImPlot::BeginPlot("Torque input vs Time", "time", "Torque Input",
                          ImVec2(-1, 200))) {
      ImPlot::PlotLine("torque input", plot_var::t_plot, plot_var::torque_plot,
                       plot_var::euler_timesteps);
      ImPlot::EndPlot();
    }

    // Beta plot
    ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, -30, 30);
    ImPlot::PushColormap(ImPlotColormap_Pastel);
    if (ImPlot::BeginPlot("Roll angle vs Time", "time", "roll angle",
                          ImVec2(-1, 200))) {
      ImPlot::PlotLine("torque input", plot_var::t_plot, plot_var::beta_plot,
                       plot_var::euler_timesteps);
      ImPlot::EndPlot();
    }

    // // Error plot
    // ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max,
    //                           -plot_var::y_max, plot_var::y_max);
    // ImPlot::PushColormap(ImPlotColormap_Pastel);
    // if (ImPlot::BeginPlot("Error vs Time", "time", "error", ImVec2(-1, 200)))
    // {
    //   ImPlot::PlotLine("error", plot_var::t_plot,
    //   plot_var::altitude_error_plot,
    //                    plot_var::euler_timesteps);
    //   ImPlot::EndPlot();
    // }

    ImGui::End();
  }
};