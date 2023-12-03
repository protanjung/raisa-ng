#include "raisa_routine/routine.hpp"

void Routine::jalan_manual(float _dx, float _dy, float _dtheta) {
  static float buffer_dx = 0, buffer_dy = 0, buffer_dtheta = 0;
  buffer_dx = 0.70 * buffer_dx + 0.30 * _dx;
  buffer_dy = 0.70 * buffer_dy + 0.30 * _dy;
  buffer_dtheta = 0.40 * buffer_dtheta + 0.60 * _dtheta;

  static const float roda_r = sqrtf(powf(raisa_roda_offset_x, 2) + powf(raisa_roda_offset_y, 2));
  static const float roda_a = atan2f(raisa_roda_offset_y, raisa_roda_offset_x);
  cmd_dx_out = buffer_dx + (buffer_dtheta * roda_r) * cosf(roda_a - M_PI_2);
  cmd_dy_out = buffer_dy + (buffer_dtheta * roda_r) * sinf(roda_a - M_PI_2);
  cmd_dtheta_out = buffer_dtheta;
}

void Routine::obstacle_influence(
    float _dx, float _dy, float _dtheta, float& _dx_out, float& _dy_out, float& _dtheta_out) {
  _dx_out = (1 - obstacle_data.emergency) * (1 - obstacle_data.velocity_following);
  _dx_out *= _dx;

  _dy_out = (1 - obstacle_data.emergency) * (obstacle_data.velocity_left - obstacle_data.velocity_right) * 0.5;
  _dy_out += _dy;

  _dtheta_out = (1 - obstacle_data.emergency) * (obstacle_data.steering_left - obstacle_data.steering_right) * 2.0;
  _dtheta_out += _dtheta;
}