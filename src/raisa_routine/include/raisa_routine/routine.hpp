#ifndef ROUTINE_HPP_
#define ROUTINE_HPP_

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pandu_ros2_kit/help_marker.hpp"
#include "raisa_interfaces/msg/basestation_from_pc.hpp"
#include "raisa_interfaces/msg/basestation_to_pc.hpp"
#include "raisa_interfaces/msg/stm32_from_pc.hpp"
#include "raisa_interfaces/msg/stm32_to_pc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define BTN_TRIGGER (button_now[0] && !button_old[0])
#define BTN_THUMB (button_now[1] && !button_old[1])
#define BTN_3 (button_now[2] && !button_old[2])
#define BTN_4 (button_now[3] && !button_old[3])
#define BTN_5 (button_now[4] && !button_old[4])
#define BTN_6 (button_now[5] && !button_old[5])
#define BTN_7 (button_now[6] && !button_old[6])
#define BTN_8 (button_now[7] && !button_old[7])
#define BTN_9 (button_now[8] && !button_old[8])
#define BTN_10 (button_now[9] && !button_old[9])
#define BTN_11 (button_now[10] && !button_old[10])
#define BTN_12 (button_now[11] && !button_old[11])
#define BTN_UP (button_now[12] && !button_old[12])
#define BTN_DOWN (button_now[13] && !button_old[13])
#define BTN_LEFT (button_now[14] && !button_old[14])
#define BTN_RIGHT (button_now[15] && !button_old[15])
#define BTN_START (button_now[16] && !button_old[16])
#define BTN_SELECT (button_now[17] && !button_old[17])

#define MAP(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class Routine : public rclcpp::Node {
 public:
  //-----Parameter
  double raisa_conversion_stm32_from_pc_linear_multiplier;
  double raisa_conversion_stm32_from_pc_angular_multiplier;
  double raisa_roda_offset_x;
  double raisa_roda_offset_y;
  double raisa_body_width;
  double raisa_body_length;
  double raisa_body_height;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_10hz;
  rclcpp::TimerBase::SharedPtr tim_50hz;
  //-----Subscriber
  rclcpp::Subscription<raisa_interfaces::msg::Stm32ToPc>::SharedPtr sub_stm32_to_pc;
  rclcpp::Subscription<raisa_interfaces::msg::BasestationToPc>::SharedPtr sub_basestation_to_pc;
  //-----Publisher
  rclcpp::Publisher<raisa_interfaces::msg::Stm32FromPc>::SharedPtr pub_stm32_from_pc;
  rclcpp::Publisher<raisa_interfaces::msg::BasestationFromPc>::SharedPtr pub_basestation_from_pc;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose;
  //-----Service client
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli_pose_reset;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli_rtabmap_reset;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli_rtabmap_set_mode_localization;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli_rtabmap_set_mode_mapping;
  //-----Help
  HelpMarker _marker;

  // STM32 and basestaion data
  // =========================
  raisa_interfaces::msg::Stm32FromPc stm32_from_pc;
  raisa_interfaces::msg::Stm32ToPc stm32_to_pc;
  raisa_interfaces::msg::BasestationFromPc basestation_from_pc;
  raisa_interfaces::msg::BasestationToPc basestation_to_pc;

  // Robot pose input and output
  // ===========================
  float cmd_dx_in, cmd_dy_in, cmd_dtheta_in;
  float cmd_dx_out, cmd_dy_out, cmd_dtheta_out;
  float fb_x, fb_y, fb_theta;
  float fb_dx, fb_dy, fb_dtheta;

  // Status algoritma
  // ================
  bool status_allow_motion = false;
  bool status_battery_charging = false;
  int algorithm_mission = 0;
  int algorithm_storage = 0;

  // Button
  // ======
  bool button_now[18] = {0};
  bool button_old[18] = {0};

  Routine();

  void cllbck_tim_10hz();
  void cllbck_tim_50hz();

  void cllbck_sub_stm32_to_pc(raisa_interfaces::msg::Stm32ToPc::SharedPtr msg);
  void cllbck_sub_basestaion_to_pc(raisa_interfaces::msg::BasestationToPc::SharedPtr msg);

  void process_all();
  void process_marker();
  void process_storage();
  void process_mission();

  void jalan_manual(float _dx, float _dy, float _dtheta, float _a_linear = 2.0, float _a_angular = M_PI);

  void publish_initialpose(float _x, float _y, float _theta);

  bool slam_reset();
  bool slam_localization_mode();
  bool slam_mapping_mode();

  geometry_msgs::msg::Quaternion rpy_to_quaternion(float _roll, float _pitch, float _yaw);
};

#endif