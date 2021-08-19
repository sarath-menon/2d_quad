// #include "pid.h"
#include "quad_2d.h"

int main() {
  Quad2D quad;

  for (;;) {
    quad.sensor_read();
    quad.dynamics();
    quad.euler_step();
  }
}