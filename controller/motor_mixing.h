
float motor_mixing(float motor_speeds[4], const float thrust_command,
                   const float torque_command, const float k_f,
                   const float arm_length) {
  float f2 = (thrust_command / 4) + (torque_command / (2 * arm_length));
  float f4 = thrust_command - f2;

  // In plane motors
  motor_speeds[1] = (f2) / (k_f * k_f);
  motor_speeds[3] = (f4) / (k_f * k_f);

  // Our of plane motor speeds set to averge of in plane
  float f1 = (f2 + f4) / 2;
  float f3 = f1;
  motor_speeds[0] = (f1) / (k_f * k_f);
  motor_speeds[2] = (f3) / (k_f * k_f);
}
