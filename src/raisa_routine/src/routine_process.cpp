#include "raisa_routine/routine.hpp"

void Routine::process_all() {
  for (int i = 0; i < 16; i++) {
    button_old[i] = button_now[i];
    button_now[i] = basestation_to_pc.tombol & (1 << i);
  }
  for (int i = 0; i < 2; i++) {
    button_old[i + 16] = button_now[i + 16];
    button_now[i + 16] = stm32_to_pc.tombol & (1 << i);
  }

  process_marker();
  process_storage();
  process_mission();
}

// =============================================================================
// -----------------------------------------------------------------------------
// =============================================================================

void Routine::process_marker() {
  geometry_msgs::msg::Point p;

  _marker.cube(
      "body_link",
      "body",
      1,
      p,
      rpy_to_quaternion(0, 0, 0),
      std::vector<float>{0, 0, 0, 0.5},
      raisa_body_length,
      raisa_body_width,
      raisa_body_height);
}

// =============================================================================
// -----------------------------------------------------------------------------
// =============================================================================

void Routine::process_storage() {}

// =============================================================================
// -----------------------------------------------------------------------------
// =============================================================================

void Routine::process_mission() {
  float tgt_dx = 0, tgt_dy = 0, tgt_dtheta = 0;
  float tgt_a_linear = 2.0, tgt_a_angular = M_PI;

  static rclcpp::Time time_old = this->now();
  static rclcpp::Time time_now = this->now();
  time_old = time_now;
  time_now = this->now();
  double dt = (time_now - time_old).seconds();

  // ===================================

  switch (algorithm_mission) {
    // IDDLE: User can move around the robot manually
    case 0: {
      tgt_dx = cmd_dx_in * 0.8;
      tgt_dy = cmd_dy_in * 0.2;
      tgt_dtheta = cmd_dtheta_in * 1.0;

      if (BTN_8) {
        publish_initialpose(0, 0, 0);
        RCLCPP_WARN(this->get_logger(), "Robot restarted at origin (0, 0, 0). Be aware of wrong position.");
      }

      if (BTN_7) {
        if (slam_mapping_mode() == false) { break; }
        algorithm_mission = 1;
        RCLCPP_WARN(this->get_logger(), "State 0 (IDDLE) -> 1 (MAPPING MODE)");
      }

      break;
    }
    // MAPPING MODE: User operate the robot to map the environment
    case 1: {
      tgt_dx = cmd_dx_in * 0.8 * 1.25;
      tgt_dy = cmd_dy_in * 0.2 * 1.25;
      tgt_dtheta = cmd_dtheta_in * 1.0 * 1.25;

      if (BTN_8) {
        if (slam_reset() == false) { break; }
        RCLCPP_WARN(this->get_logger(), "Mapping restarted at origin (0, 0, 0). You can start mapping now.");
      }

      if (BTN_7) {
        if (slam_localization_mode() == false) { break; }
        algorithm_mission = 0;
        RCLCPP_WARN(this->get_logger(), "State 1 (MAPPING MODE) -> 0 (IDDLE)");
      }

      break;
    }
  }

  // ===================================

  jalan_manual(tgt_dx, tgt_dy, tgt_dtheta, tgt_a_linear, tgt_a_angular);
}

// =============================================================================
// -----------------------------------------------------------------------------
// =============================================================================

void Routine::publish_initialpose(float _x, float _y, float _theta) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg_initialpose;
  msg_initialpose.header.frame_id = "map";
  msg_initialpose.header.stamp = this->now();
  msg_initialpose.pose.pose.position.x = _x;
  msg_initialpose.pose.pose.position.y = _y;
  msg_initialpose.pose.pose.position.z = 0;
  msg_initialpose.pose.pose.orientation = rpy_to_quaternion(0, 0, _theta);
  msg_initialpose.pose.covariance[0] = 1e-12;
  msg_initialpose.pose.covariance[7] = 1e-12;
  msg_initialpose.pose.covariance[14] = 1e6;
  msg_initialpose.pose.covariance[21] = 1e6;
  msg_initialpose.pose.covariance[28] = 1e6;
  msg_initialpose.pose.covariance[35] = 1e-12;
  pub_initialpose->publish(msg_initialpose);
}

bool Routine ::slam_reset() {
  if (!cli_pose_reset->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Service pose/reset not available");
    return false;
  }
  if (!cli_rtabmap_reset->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Service rtabmap/reset not available");
    return false;
  }
  auto req_reset = std::make_shared<std_srvs::srv::Empty::Request>();
  cli_pose_reset->async_send_request(req_reset);
  cli_rtabmap_reset->async_send_request(req_reset);
  return true;
}

bool Routine::slam_localization_mode() {
  if (!cli_rtabmap_set_mode_localization->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Service rtabmap/set_mode_localization not available");
    return false;
  }
  auto req_set_mode_localization = std::make_shared<std_srvs::srv::Empty::Request>();
  cli_rtabmap_set_mode_localization->async_send_request(req_set_mode_localization);
  return true;
}

bool Routine::slam_mapping_mode() {
  if (!cli_rtabmap_set_mode_mapping->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Service rtabmap/set_mode_mapping not available");
    return false;
  }
  auto req_set_mode_mapping = std::make_shared<std_srvs::srv::Empty::Request>();
  cli_rtabmap_set_mode_mapping->async_send_request(req_set_mode_mapping);
  return true;
}

geometry_msgs::msg::Quaternion Routine::rpy_to_quaternion(float _roll, float _pitch, float _yaw) {
  tf2::Quaternion q_in;
  geometry_msgs::msg::Quaternion q_out;

  q_in.setRPY(_roll, _pitch, _yaw);
  tf2::convert(q_in, q_out);

  return q_out;
}