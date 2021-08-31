#pragma once
#include <iostream>
#include <math.h>

void motor_mixing(float motor_speeds[4], const float thrust_command,
                  const float torque_command, const float k_f,
                  const float arm_length) {
  float f2 = (thrust_command / 4) + (torque_command / (2 * arm_length));
  float f4 = (thrust_command / 4) - (torque_command / (2 * arm_length));

  // In plane motors
  motor_speeds[1] = sqrt(f2 / k_f * k_f);
  motor_speeds[3] = sqrt(f4 / k_f * k_f);

  // Our of plane motor speeds set to averge of in plane
  float f1 = (f2 + f4) / 2;
  float f3 = f1;

  motor_speeds[0] = sqrt(f1 / k_f * k_f);
  motor_speeds[2] = sqrt(f3 / k_f * k_f);

  // std::cout << "f1:" << f1 << "\tf2:" << f2 << "\tf3:" << f3 << "\tf4:" <<
  // f4 << std::endl;
  // std::cout << "Net thrust before and after motor mixing:" << thrust_command
  << '\t' << f1 + f2 + f3 + f4 << std::endl;
  // std::cout << "Net torque before and after motor mixing:" << torque_command
  << '\t' << (f2 - f4) * arm_length << std::endl;
}
