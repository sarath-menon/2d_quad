#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>

using namespace mahi::gui;
using namespace mahi::util;

namespace plot_var {
const int euler_timesteps = 100;
float xs[euler_timesteps], ys[euler_timesteps];
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
    ImPlot::SetNextPlotLimits(0, 100, 0, 50);

    if (ImPlot::BeginPlot("Shaded Plots", "X-Axis", "Y-Axis")) {
      ImPlot::PlotLine("Uncertain Data", plot_var::xs, plot_var::ys,
                       plot_var::euler_timesteps);
      ImPlot::EndPlot();
    }
    ImGui::End();
  }
};