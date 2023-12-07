#ifndef ROUTINE_HPP_
#define ROUTINE_HPP_

#include "boost/filesystem.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pandu_ros2_kit/help_marker.hpp"
#include "pandu_ros2_kit/pid.hpp"
#include "pandu_ros2_kit/pure_pursuit.hpp"
#include "raisa_interfaces/msg/basestation_from_pc.hpp"
#include "raisa_interfaces/msg/basestation_to_pc.hpp"
#include "raisa_interfaces/msg/faces.hpp"
#include "raisa_interfaces/msg/obstacle_data.hpp"
#include "raisa_interfaces/msg/obstacle_parameter.hpp"
#include "raisa_interfaces/msg/stm32_from_pc.hpp"
#include "raisa_interfaces/msg/stm32_to_pc.hpp"
#include "raisa_interfaces/msg/ui_from_pc.hpp"
#include "raisa_interfaces/msg/ui_to_pc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define BDN_TRIGGER (button_now[0] && !button_old[0])
#define BDN_THUMB (button_now[1] && !button_old[1])
#define BDN_3 (button_now[2] && !button_old[2])
#define BDN_4 (button_now[3] && !button_old[3])
#define BDN_5 (button_now[4] && !button_old[4])
#define BDN_6 (button_now[5] && !button_old[5])
#define BDN_7 (button_now[6] && !button_old[6])
#define BDN_8 (button_now[7] && !button_old[7])
#define BDN_9 (button_now[8] && !button_old[8])
#define BDN_10 (button_now[9] && !button_old[9])
#define BDN_11 (button_now[10] && !button_old[10])
#define BDN_12 (button_now[11] && !button_old[11])
#define BDN_UP (button_now[12] && !button_old[12])
#define BDN_DOWN (button_now[13] && !button_old[13])
#define BDN_LEFT (button_now[14] && !button_old[14])
#define BDN_RIGHT (button_now[15] && !button_old[15])
#define BDN_START (button_now[16] && !button_old[16])
#define BDN_SELECT (button_now[17] && !button_old[17])
#define BUP_TRIGGER (!button_now[0] && button_old[0])
#define BUP_THUMB (!button_now[1] && button_old[1])
#define BUP_3 (!button_now[2] && button_old[2])
#define BUP_4 (!button_now[3] && button_old[3])
#define BUP_5 (!button_now[4] && button_old[4])
#define BUP_6 (!button_now[5] && button_old[5])
#define BUP_7 (!button_now[6] && button_old[6])
#define BUP_8 (!button_now[7] && button_old[7])
#define BUP_9 (!button_now[8] && button_old[8])
#define BUP_10 (!button_now[9] && button_old[9])
#define BUP_11 (!button_now[10] && button_old[10])
#define BUP_12 (!button_now[11] && button_old[11])
#define BUP_UP (!button_now[12] && button_old[12])
#define BUP_DOWN (!button_now[13] && button_old[13])
#define BUP_LEFT (!button_now[14] && button_old[14])
#define BUP_RIGHT (!button_now[15] && button_old[15])
#define BUP_START (!button_now[16] && button_old[16])
#define BUP_SELECT (!button_now[17] && button_old[17])

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
  std::string raisa_path_misc;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_10hz;
  rclcpp::TimerBase::SharedPtr tim_50hz;
  //-----Subscriber
  rclcpp::Subscription<raisa_interfaces::msg::Stm32ToPc>::SharedPtr sub_stm32_to_pc;
  rclcpp::Subscription<raisa_interfaces::msg::BasestationToPc>::SharedPtr sub_basestation_to_pc;
  rclcpp::Subscription<raisa_interfaces::msg::UiToPc>::SharedPtr sub_ui_to_pc;
  rclcpp::Subscription<raisa_interfaces::msg::ObstacleData>::SharedPtr sub_obstacle_data;
  rclcpp::Subscription<raisa_interfaces::msg::Faces>::SharedPtr sub_faces;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_filtered;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_thermal;
  //-----Publisher
  rclcpp::Publisher<raisa_interfaces::msg::Stm32FromPc>::SharedPtr pub_stm32_from_pc;
  rclcpp::Publisher<raisa_interfaces::msg::BasestationFromPc>::SharedPtr pub_basestation_from_pc;
  rclcpp::Publisher<raisa_interfaces::msg::UiFromPc>::SharedPtr pub_ui_from_pc;
  rclcpp::Publisher<raisa_interfaces::msg::ObstacleParameter>::SharedPtr pub_obstacle_parameter;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose;
  //-----Service client
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli_pose_reset;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli_rtabmap_reset;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli_rtabmap_set_mode_localization;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli_rtabmap_set_mode_mapping;
  //-----Help
  HelpMarker _marker;

  // STM32, basestation, and UI data
  // ===============================
  raisa_interfaces::msg::Stm32FromPc stm32_from_pc;
  raisa_interfaces::msg::Stm32ToPc stm32_to_pc;
  raisa_interfaces::msg::BasestationFromPc basestation_from_pc;
  raisa_interfaces::msg::BasestationToPc basestation_to_pc;
  raisa_interfaces::msg::UiFromPc ui_from_pc;
  raisa_interfaces::msg::UiToPc ui_to_pc;

  // Robot pose input and output
  // ===========================
  float cmd_dx_in, cmd_dy_in, cmd_dtheta_in;
  float cmd_dx_out, cmd_dy_out, cmd_dtheta_out;
  float fb_x, fb_y, fb_theta;
  float fb_dx, fb_dy, fb_dtheta;

  // Obstacle data and parameter
  // ===========================
  raisa_interfaces::msg::ObstacleData obstacle_data;
  raisa_interfaces::msg::ObstacleParameter obstacle_parameter;

  // Status algoritma
  // ================
  int algorithm_mission = 0;
  int algorithm_storage = 0;

  // Button
  // ======
  bool button_now[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1};
  bool button_old[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1};

  // Vision
  // ======
  uint8_t human_presence = 0;
  float human_position = 0;
  float human_temperature = 0;

  // Route and POI
  // =============
  typedef struct {
    float x;
    float y;
  } route;
  typedef struct {
    float x;
    float y;
    float theta;
    float duration;
    std::vector<geometry_msgs::msg::Point> list_route_entry;
    std::vector<geometry_msgs::msg::Point> list_route_exit;
    float gate_x;
    float gate_y;
  } poi;
  std::vector<route> list_route;
  std::vector<poi> list_poi;

  // Path and pure pursuit
  // =====================
  std::vector<geometry_msgs::msg::Point> path_active, path_kiri, path_kanan;
  PurePursuit pp_active, pp_kiri, pp_kanan;

  Routine();

  void cllbck_tim_10hz();
  void cllbck_tim_50hz();

  void cllbck_sub_stm32_to_pc(const raisa_interfaces::msg::Stm32ToPc::SharedPtr msg);
  void cllbck_sub_basestaion_to_pc(const raisa_interfaces::msg::BasestationToPc::SharedPtr msg);
  void cllbck_sub_ui_to_pc(const raisa_interfaces::msg::UiToPc::SharedPtr msg);
  void cllbck_sub_obstacle_data(const raisa_interfaces::msg::ObstacleData::SharedPtr msg);
  void cllbck_sub_faces(const raisa_interfaces::msg::Faces::SharedPtr msg);
  void cllbck_sub_odometry_filtered(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cllbck_sub_thermal(const std_msgs::msg::Float32::SharedPtr msg);

  //====================================

  void jalan_manual(float _dx, float _dy, float _dtheta);
  void jalan_manual_lapangan(float _dx, float _dy, float _dtheta);

  bool jalan_posisi_sudut(
      float _target_x,
      float _target_y,
      float _target_sudut,
      float& _output_dx,
      float& _output_dy,
      float& _output_dtheta,
      float _vposisi,
      float _vsudut);

  float sudut_robot_ke_titik(float _x, float _y);
  float sudut_titik_ke_titik(float _x0, float _y0, float _x1, float _y1);
  float error_sudut_robot_ke_titik(float _x, float _y);

  void obstacle_influence(float _dx, float _dy, float _dtheta, float& _dx_out, float& _dy_out, float& _dtheta_out);

  //====================================

  void process_all();
  void process_marker();
  void process_storage();
  void process_mission();

  int is_inside_poi(float _radius_offset = 0);
  int is_inside_gate(float _radius_offset = 0);

  geometry_msgs::msg::Point nearest_path(float _x, float _y, std::vector<geometry_msgs::msg::Point> _path);
  std::vector<geometry_msgs::msg::Point> generate_path(float _x0, float _y0, float _x1, float _y1, float _res = 0.1);

  bool slam_reset();
  bool slam_localization_mode();
  bool slam_mapping_mode();

  geometry_msgs::msg::Quaternion rpy_to_quaternion(float _roll, float _pitch, float _yaw);
  void publish_initialpose(float _x, float _y, float _theta);
};

#endif  // ROUTINE_HPP_