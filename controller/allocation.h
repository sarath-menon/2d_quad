
void allocation(float thrust_command, float torque_command, float arm_length) {
  float f1 = (thrust_command / 4) + (torque_command / (2 * arm_length));
  float f2 = thrust_command - f1;
}
