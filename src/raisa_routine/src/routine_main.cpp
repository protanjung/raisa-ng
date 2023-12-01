#include "raisa_routine/routine.hpp"

using namespace std::chrono_literals;

Routine::Routine() : Node("routine") {
  //-----Parameter
  this->declare_parameter("raisa.conversion.stm32_from_pc_linear_multiplier", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raisa.conversion.stm32_from_pc_angular_multiplier", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raisa.roda.offset_x", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raisa.roda.offset_y", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raisa.body.width", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raisa.body.length", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raisa.body.height", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raisa.path.misc", rclcpp::PARAMETER_STRING);
  this->get_parameter(
      "raisa.conversion.stm32_from_pc_linear_multiplier", raisa_conversion_stm32_from_pc_linear_multiplier);
  this->get_parameter(
      "raisa.conversion.stm32_from_pc_angular_multiplier", raisa_conversion_stm32_from_pc_angular_multiplier);
  this->get_parameter("raisa.roda.offset_x", raisa_roda_offset_x);
  this->get_parameter("raisa.roda.offset_y", raisa_roda_offset_y);
  this->get_parameter("raisa.body.width", raisa_body_width);
  this->get_parameter("raisa.body.length", raisa_body_length);
  this->get_parameter("raisa.body.height", raisa_body_height);
  this->get_parameter("raisa.path.misc", raisa_path_misc);
  //-----Timer
  tim_10hz = this->create_wall_timer(100ms, std::bind(&Routine::cllbck_tim_10hz, this));
  tim_50hz = this->create_wall_timer(20ms, std::bind(&Routine::cllbck_tim_50hz, this));
  //-----Subscriber
  sub_stm32_to_pc = this->create_subscription<raisa_interfaces::msg::Stm32ToPc>(
      "stm32/to_pc", 10, std::bind(&Routine::cllbck_sub_stm32_to_pc, this, std::placeholders::_1));
  sub_basestation_to_pc = this->create_subscription<raisa_interfaces::msg::BasestationToPc>(
      "basestation/to_pc", 10, std::bind(&Routine::cllbck_sub_basestaion_to_pc, this, std::placeholders::_1));
  sub_odometry_filtered = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10, std::bind(&Routine::cllbck_sub_odometry_filtered, this, std::placeholders::_1));
  //-----Publisher
  pub_stm32_from_pc = this->create_publisher<raisa_interfaces::msg::Stm32FromPc>("stm32/from_pc", 10);
  pub_basestation_from_pc = this->create_publisher<raisa_interfaces::msg::BasestationFromPc>("basestation/from_pc", 10);
  pub_initialpose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
  //-----Service client
  cli_pose_reset = this->create_client<std_srvs::srv::Empty>("pose/reset");
  cli_rtabmap_reset = this->create_client<std_srvs::srv::Empty>("rtabmap/reset");
  cli_rtabmap_set_mode_localization = this->create_client<std_srvs::srv::Empty>("rtabmap/set_mode_localization");
  cli_rtabmap_set_mode_mapping = this->create_client<std_srvs::srv::Empty>("rtabmap/set_mode_mapping");
}

//====================================

void Routine::cllbck_tim_10hz() {
  if (!_marker.is_initialized()) { _marker.init(this->shared_from_this()); }

  if (!pp_active.is_initialized()) { pp_active.init(&fb_x, &fb_y, &fb_theta, &path_active, 0.25, 0.50); }

  process_all();
}

void Routine::cllbck_tim_50hz() {
  bool temp_status_allow_motion = true;
  if (status_battery_charging) { temp_status_allow_motion = false; }
  status_allow_motion = temp_status_allow_motion;

  // ===================================

  stm32_from_pc.mode = status_allow_motion ? 1 : 0;
  stm32_from_pc.dx = cmd_dx_out * raisa_conversion_stm32_from_pc_linear_multiplier;
  stm32_from_pc.dy = cmd_dy_out * raisa_conversion_stm32_from_pc_linear_multiplier;
  stm32_from_pc.dtheta = cmd_dtheta_out * raisa_conversion_stm32_from_pc_angular_multiplier;
  pub_stm32_from_pc->publish(stm32_from_pc);

  basestation_from_pc.status = algorithm_mission;
  basestation_from_pc.x = fb_x;
  basestation_from_pc.y = fb_y;
  basestation_from_pc.theta = fb_theta;
  basestation_from_pc.battery_voltage = stm32_to_pc.battery_voltage;
  basestation_from_pc.battery_current = stm32_to_pc.battery_current;
  basestation_from_pc.battery_soc = stm32_to_pc.battery_soc;
  basestation_from_pc.battery_charging = stm32_to_pc.battery_charging;
  pub_basestation_from_pc->publish(basestation_from_pc);
}

//====================================

void Routine::cllbck_sub_stm32_to_pc(raisa_interfaces::msg::Stm32ToPc::SharedPtr msg) {
  stm32_to_pc = *msg;

  status_battery_charging = msg->battery_charging;
}

void Routine::cllbck_sub_basestaion_to_pc(raisa_interfaces::msg::BasestationToPc::SharedPtr msg) {
  basestation_to_pc = *msg;

  cmd_dx_in = abs(-msg->dy) < 1000 ? 0
              : -msg->dy < 0       ? MAP((float)-msg->dy, -1000, -10000, 0, -1)
                                   : MAP((float)-msg->dy, 1000, 10000, 0, 1);
  cmd_dy_in = abs(-msg->dx) < 1000 ? 0
              : -msg->dx < 0       ? MAP((float)-msg->dx, -1000, -10000, 0, -1)
                                   : MAP((float)-msg->dx, 1000, 10000, 0, 1);
  cmd_dtheta_in = abs(-msg->dtheta) < 3000 ? 0
                  : -msg->dtheta < 0       ? MAP((float)-msg->dtheta, -3000, -10000, 0, -1)
                                           : MAP((float)-msg->dtheta, 3000, 10000, 0, 1);
}

void Routine::cllbck_sub_odometry_filtered(nav_msgs::msg::Odometry::SharedPtr msg) {
  tf2::Quaternion q;
  double roll, pitch, yaw;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  fb_x = msg->pose.pose.position.x;
  fb_y = msg->pose.pose.position.y;
  fb_theta = yaw;

  fb_dx = msg->twist.twist.linear.x;
  fb_dy = msg->twist.twist.linear.y;
  fb_dtheta = msg->twist.twist.angular.z;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node_routine = std::make_shared<Routine>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_routine);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
