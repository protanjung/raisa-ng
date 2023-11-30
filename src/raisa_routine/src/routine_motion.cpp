#include "raisa_routine/routine.hpp"

void Routine::jalan_manual(float _dx, float _dy, float _dtheta, float _a_linear, float _a_angular) {
  static rclcpp::Time time_old = this->now();
  static rclcpp::Time time_now = this->now();
  time_old = time_now;
  time_now = this->now();
  double dt = (time_now - time_old).seconds();

  static float buffer_dx = 0, buffer_dy = 0, buffer_dtheta = 0;
  if (buffer_dx < _dx) {
    buffer_dx = fminf(buffer_dx + _a_linear * dt, _dx);
  } else if (buffer_dx > _dx) {
    buffer_dx = fmaxf(buffer_dx - _a_linear * dt, _dx);
  }
  if (buffer_dy < _dy) {
    buffer_dy = fminf(buffer_dy + _a_linear * dt, _dy);
  } else if (buffer_dy > _dy) {
    buffer_dy = fmaxf(buffer_dy - _a_linear * dt, _dy);
  }
  if (buffer_dtheta < _dtheta) {
    buffer_dtheta = fminf(buffer_dtheta + _a_angular * dt, _dtheta);
  } else if (buffer_dtheta > _dtheta) {
    buffer_dtheta = fmaxf(buffer_dtheta - _a_angular * dt, _dtheta);
  }

  static const float roda_r = sqrtf(powf(raisa_roda_offset_x, 2) + powf(raisa_roda_offset_y, 2));
  static const float roda_a = atan2f(raisa_roda_offset_y, raisa_roda_offset_x);
  cmd_dx_out = buffer_dx + (buffer_dtheta * roda_r) * cosf(roda_a - M_PI_2);
  cmd_dy_out = buffer_dy + (buffer_dtheta * roda_r) * sinf(roda_a - M_PI_2);
  cmd_dtheta_out = buffer_dtheta;
}