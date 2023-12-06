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

void Routine::jalan_manual_lapangan(float _dx, float _dy, float _dtheta) {
  static float buffer_dx = 0, buffer_dy = 0;
  buffer_dx = _dx * cosf(fb_theta) + _dy * sinf(fb_theta);
  buffer_dy = _dx * -sinf(fb_theta) + _dy * cosf(fb_theta);

  jalan_manual(buffer_dx, buffer_dy, _dtheta);
}

// =============================================================================

bool Routine::jalan_posisi_sudut(
    float _target_x,
    float _target_y,
    float _target_sudut,
    float& _output_dx,
    float& _output_dy,
    float& _output_dtheta,
    float _vposisi,
    float _vsudut) {
  static PID pid_posisi;
  static PID pid_sudut;

  static bool first = true;
  if (first) {
    pid_posisi.init(1.0, 0.0, 0.0, 0.1);
    pid_sudut.init(1.0, 0.0, 0.0, 0.1);
    first = false;
  }

  float error_x = _target_x - fb_x;
  float error_y = _target_y - fb_y;
  float error_posisi = sqrt(pow(error_x, 2) + pow(error_y, 2));

  float error_sudut = _target_sudut - fb_theta;
  while (error_sudut < -M_PI) { error_sudut += 2 * M_PI; }
  while (error_sudut > M_PI) { error_sudut -= 2 * M_PI; }

  float output_posisi = pid_posisi.update(error_posisi, -_vposisi, _vposisi);
  float output_sudut = pid_sudut.update(error_sudut, -_vsudut, _vsudut);

  float output_x = output_posisi * cos(atan2(error_y, error_x));
  float output_y = output_posisi * sin(atan2(error_y, error_x));

  _output_dx = output_x;
  _output_dy = output_y;
  _output_dtheta = output_sudut;

  if (error_posisi < 0.13 && fabs(error_sudut) < 0.0436332) {
    return true;
  } else {
    return false;
  }
}

// =============================================================================

float Routine::sudut_robot_ke_titik(float _x, float _y) { return atan2f(_y - fb_y, _x - fb_x); }

float Routine::sudut_titik_ke_titik(float _x0, float _y0, float _x1, float _y1) { return atan2f(_y1 - _y0, _x1 - _x0); }

float Routine::error_sudut_robot_ke_titik(float _x, float _y) {
  float error = sudut_robot_ke_titik(_x, _y) - fb_theta;
  if (error > M_PI) {
    error -= 2 * M_PI;
  } else if (error < -M_PI) {
    error += 2 * M_PI;
  }
  return error;
}

// =============================================================================

void Routine::obstacle_influence(
    float _dx, float _dy, float _dtheta, float& _dx_out, float& _dy_out, float& _dtheta_out) {
  _dx_out = (1 - obstacle_data.emergency) * (1 - obstacle_data.velocity_following);
  _dx_out *= _dx;

  _dy_out = (1 - obstacle_data.emergency) * (obstacle_data.velocity_left - obstacle_data.velocity_right) * 0.5;
  _dy_out += _dy;

  _dtheta_out = (1 - obstacle_data.emergency) * (obstacle_data.steering_left - obstacle_data.steering_right) * 2.0;
  _dtheta_out += _dtheta;
}