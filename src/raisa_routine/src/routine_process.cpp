#include "raisa_routine/routine.hpp"

void Routine::process_all() {
  process_marker();
  process_storage();
  process_mission();
}

// =============================================================================
// -----------------------------------------------------------------------------
// =============================================================================

void Routine::process_marker() {
  std::vector<geometry_msgs::msg::Point> ps;
  geometry_msgs::msg::Point p;

  // ============
  // Body markers
  // ============
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

  // =============
  // Route markers
  // =============
  ps.clear();
  for (auto i : list_route) {
    p.x = i.x;
    p.y = i.y;
    ps.push_back(p);
  }
  _marker.sphere_list(
      "map", "route", 1, ps, rpy_to_quaternion(0, 0, 0), std::vector<float>{0, 1, 1, 0.5}, 0.2, 0.2, 0.05);

  // ===========
  // POI markers
  // ===========
  ps.clear();
  for (auto i : list_poi) {
    p.x = i.x;
    p.y = i.y;
    ps.push_back(p);
  }
  _marker.sphere_list(
      "map", "poi", 1, ps, rpy_to_quaternion(0, 0, 0), std::vector<float>{0, 0, 1, 0.5}, 0.3, 0.3, 0.05);

  ps.clear();
  for (auto i : list_poi) {
    p.x = i.gate_x;
    p.y = i.gate_y;
    ps.push_back(p);
  }
  _marker.sphere_list(
      "map", "poi", 2, ps, rpy_to_quaternion(0, 0, 0), std::vector<float>{0, 1, 0, 0.5}, 0.3, 0.3, 0.05);

  ps.clear();
  for (auto i : list_poi) {
    p.x = i.x;
    p.y = i.y;
    ps.push_back(p);
    p.x = i.gate_x;
    p.y = i.gate_y;
    ps.push_back(p);
  }
  _marker.line_list("map", "poi", 3, ps, std::vector<float>{0.5, 0.5, 0.5, 1}, 0.01);

  // ============
  // Path markers
  // ============
  _marker.line_strip("map", "path", 1, path_active, std::vector<float>{0, 0, 0, 1}, 0.02);

  // ====================
  // Pure pursuit markers
  // ====================
  ps.clear();
  for (int i = 0; i < 91; i++) {
    p.x = pp_active.look_ahead_distance * cos(i * M_PI / 45);
    p.y = pp_active.look_ahead_distance * sin(i * M_PI / 45);
    ps.push_back(p);
  }
  _marker.line_strip("body_link", "pp", 1, ps, std::vector<float>{0, 1, 0, 0.5}, 0.01);

  p.x = pp_active.goal_x;
  p.y = pp_active.goal_y;
  _marker.sphere("map", "pp", 2, p, rpy_to_quaternion(0, 0, 0), std::vector<float>{1, 1, 0, 0.5}, 0.1, 0.1, 0.05);
}

// =============================================================================
// -----------------------------------------------------------------------------
// =============================================================================

void Routine::process_storage() {
  static std::string route_file = raisa_path_misc + "/route.csv";
  static std::string poi_file = raisa_path_misc + "/poi.csv";

  switch (algorithm_storage) {
    // ===================================
    // LOAD: Load route and POI from files
    // ===================================
    case 0: {
      list_route.clear();
      list_poi.clear();

      if (!boost::filesystem::exists(raisa_path_misc)) { boost::filesystem::create_directories(raisa_path_misc); }
      if (!boost::filesystem::exists(route_file)) {
        std::ofstream file(route_file);
        file << "x,y\n";
        file.close();
      }
      if (!boost::filesystem::exists(poi_file)) {
        std::ofstream file(poi_file);
        file << "x,y\n";
        file.close();
      }

      std::ifstream file;

      file.open(route_file);
      file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      while (file.good()) {
        if (file.peek() == EOF) { break; }
        route x;
        file >> x.x;
        file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
        file >> x.y;
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        list_route.push_back(x);
      }
      file.close();

      file.open(poi_file);
      file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      while (file.good()) {
        if (file.peek() == EOF) { break; }
        poi x;
        file >> x.x;
        file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
        file >> x.y;
        file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
        file >> x.theta;
        file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
        file >> x.duration;
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        list_poi.push_back(x);
      }
      file.close();

      algorithm_storage = 2;
      break;
    }
    // ==================================
    // SAVE: Write route and POI to files
    // ==================================
    case 1: {
      std::ofstream file;

      file.open(route_file);
      file << "x,y\n";
      for (auto p : list_route) { file << p.x << "," << p.y << "\n"; }
      file.close();

      file.open(poi_file);
      file << "x,y,theta,duration\n";
      for (auto p : list_poi) { file << p.x << "," << p.y << "," << p.theta << "," << p.duration << "\n"; }
      file.close();

      algorithm_storage = 2;
      break;
    }
    // ==============================================
    // MISC: Do something that is supposed to be done
    // ==============================================
    case 2: {
      path_active.clear();
      for (size_t i = 1; i < list_route.size(); i++) {
        auto path = generate_path(list_route[i - 1].x, list_route[i - 1].y, list_route[i].x, list_route[i].y);
        path_active.insert(path_active.end(), path.begin(), path.end());
      }
      auto path = generate_path(list_route.back().x, list_route.back().y, list_route.front().x, list_route.front().y);
      path_active.insert(path_active.end(), path.begin(), path.end());

      for (auto& i : list_poi) {
        geometry_msgs::msg::Point p = nearest_path(i.x, i.y, path_active);
        i.gate_x = p.x;
        i.gate_y = p.y;
        i.list_route_entry = generate_path(i.x, i.y, i.gate_x, i.gate_y);
        i.list_route_exit = generate_path(i.gate_x, i.gate_y, i.x, i.y);
      }

      algorithm_storage = -1;
      break;
    }
  };
}

// =============================================================================
// -----------------------------------------------------------------------------
// =============================================================================

void Routine::process_mission() {
  static rclcpp::Time time_old = this->now();
  static rclcpp::Time time_now = this->now();
  time_old = time_now;
  time_now = this->now();
  double dt = (time_now - time_old).seconds();

  // ===================================

  // * Robot velocity target
  float tgt_dx = 0, tgt_dy = 0, tgt_dtheta = 0;

  // * Pure pursuit's look ahead distance target
  float tgt_look_ahead_distance = 0.5;
  if (pp_active.look_ahead_distance < tgt_look_ahead_distance) {
    pp_active.look_ahead_distance = fminf(pp_active.look_ahead_distance + 1.0 * dt, tgt_look_ahead_distance);
  } else if (pp_active.look_ahead_distance > tgt_look_ahead_distance) {
    pp_active.look_ahead_distance = fmaxf(pp_active.look_ahead_distance - 1.0 * dt, tgt_look_ahead_distance);
  }

  // * Obstacle detector's laser scan distance target
  float tgt_laser_scan_distance = 1.2;
  if (obstacle_parameter.laser_scan_distance < tgt_laser_scan_distance) {
    obstacle_parameter.laser_scan_distance =
        fminf(obstacle_parameter.laser_scan_distance + 1.0 * dt, tgt_laser_scan_distance);
  } else if (obstacle_parameter.laser_scan_distance > tgt_laser_scan_distance) {
    obstacle_parameter.laser_scan_distance =
        fmaxf(obstacle_parameter.laser_scan_distance - 1.0 * dt, tgt_laser_scan_distance);
  }

  // ===================================

  switch (algorithm_mission) {
    // ==============================================
    // IDDLE: User can move around the robot manually
    // ==============================================
    case 0: {
      tgt_dx = cmd_dx_in * 0.8;
      tgt_dy = cmd_dy_in * 0.2;
      tgt_dtheta = cmd_dtheta_in * 1.0;

      if (BTN_8) {
        publish_initialpose(0, 0, 0);
        RCLCPP_WARN(this->get_logger(), "Robot restarted at origin (0, 0, 0). Be aware of wrong position.");
      }

      // -------------------------------

      if (BTN_7) {
        if (slam_mapping_mode() == false) { break; }
        algorithm_mission = 1;
        RCLCPP_WARN(this->get_logger(), "State 0 (IDDLE) -> 1 (MAPPING MODE)");
      }
      if (BTN_9) {
        algorithm_storage = 0;
        algorithm_mission = 2;
        RCLCPP_WARN(this->get_logger(), "State 0 (IDDLE) -> 2 (ROUTING MODE)");
      }
      if (BTN_11) {
        algorithm_mission = 3;
        RCLCPP_WARN(this->get_logger(), "State 0 (IDDLE) -> 3 (OPERATION MODE)");
      }

      break;
    }
    // ===========================================================
    // MAPPING MODE: User operate the robot to map the environment
    // ===========================================================
    case 1: {
      tgt_dx = cmd_dx_in * 0.8;
      tgt_dy = cmd_dy_in * 0.2;
      tgt_dtheta = cmd_dtheta_in * 1.0;

      if (BTN_8) {
        if (slam_reset() == false) { break; }
        RCLCPP_WARN(this->get_logger(), "Mapping restarted at origin (0, 0, 0). You can start mapping now.");
      }

      // -------------------------------

      if (BTN_7) {
        if (slam_localization_mode() == false) { break; }
        algorithm_mission = 0;
        RCLCPP_WARN(this->get_logger(), "State 1 (MAPPING MODE) -> 0 (IDDLE)");
      }

      break;
    }
    // =============================================================================
    // ROUTING MODE: User operate the robot to create motion route and mark the POIs
    // =============================================================================
    case 2: {
      tgt_dx = cmd_dx_in * 0.8;
      tgt_dy = cmd_dy_in * 0.2;
      tgt_dtheta = cmd_dtheta_in * 1.0;

      if (BTN_8) {
        list_route.clear();
        list_poi.clear();
        RCLCPP_WARN(this->get_logger(), "Route and POI list cleared. Please create new route and POI.");
      }
      if (BTN_9) {
        route x;
        x.x = fb_x;
        x.y = fb_y;
        list_route.push_back(x);
        RCLCPP_WARN(this->get_logger(), "Route added. There are %ld route coordinates now.", list_route.size());
      }
      if (BTN_10) {
        if (list_route.size() > 0) {
          list_route.pop_back();
          RCLCPP_WARN(this->get_logger(), "Route removed. There are %ld route coordinates now.", list_route.size());
        } else {
          RCLCPP_WARN(this->get_logger(), "Route list is empty. Cannot remove route coordinate.");
        }
      }
      if (BTN_11) {
        poi x;
        x.x = fb_x;
        x.y = fb_y;
        x.theta = fb_theta;
        if (list_poi.empty()) {
          x.duration = 10;
        } else {
          x.duration = 5;
        }
        list_poi.push_back(x);
        RCLCPP_WARN(this->get_logger(), "POI added. There are %ld POI coordinates now.", list_poi.size());
      }
      if (BTN_12) {
        if (list_poi.size() > 0) {
          list_poi.pop_back();
          RCLCPP_WARN(this->get_logger(), "POI removed. There are %ld POI coordinates now.", list_poi.size());
        } else {
          RCLCPP_WARN(this->get_logger(), "POI list is empty. Cannot remove POI coordinate.");
        }
      }

      // -------------------------------

      if (BTN_7) {
        algorithm_storage = 1;
        algorithm_mission = 0;
        RCLCPP_WARN(this->get_logger(), "State 2 (ROUTING MODE) -> 0 (IDDLE)");
      }

      break;
    }
    // =============================================================
    // OPERATTION MODE: Robot operate autonomously to do the mission
    // =============================================================
    case 3: {
      obstacle_influence(0.2, 0.0, pp_active.steering_angle, tgt_dx, tgt_dy, tgt_dtheta);

      // -------------------------------

      if (BTN_7) {
        algorithm_mission = 0;
        RCLCPP_WARN(this->get_logger(), "State 3 (OPERATION MODE) -> 0 (IDDLE)");
      }

      break;
    }
  }

  // ===================================

  jalan_manual(tgt_dx, tgt_dy, tgt_dtheta);
}

// =============================================================================
// -----------------------------------------------------------------------------
// =============================================================================

geometry_msgs::msg::Point Routine::nearest_path(float _x, float _y, std::vector<geometry_msgs::msg::Point> _path) {
  geometry_msgs::msg::Point p;
  float d_min = __FLT_MAX__;
  for (auto i : _path) {
    float d = sqrt(pow(i.x - _x, 2) + pow(i.y - _y, 2));
    if (d < d_min) {
      d_min = d;
      p.x = i.x;
      p.y = i.y;
    }
  }
  return p;
}

std::vector<geometry_msgs::msg::Point> Routine::generate_path(float _x0, float _y0, float _x1, float _y1, float _res) {
  std::vector<geometry_msgs::msg::Point> _path;

  geometry_msgs::msg::Point p;
  p.x = _x0;
  p.y = _y0;
  _path.push_back(p);

  float dx = _x1 - _x0;
  float dy = _y1 - _y0;
  float d = sqrt(dx * dx + dy * dy);
  int n = d / _res;
  for (int j = 1; j <= n; j++) {
    p.x = _x0 + dx * j / n;
    p.y = _y0 + dy * j / n;
    _path.push_back(p);
  }

  return _path;
}

// =============================================================================

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

// =============================================================================

geometry_msgs::msg::Quaternion Routine::rpy_to_quaternion(float _roll, float _pitch, float _yaw) {
  tf2::Quaternion q_in;
  geometry_msgs::msg::Quaternion q_out;

  q_in.setRPY(_roll, _pitch, _yaw);
  tf2::convert(q_in, q_out);

  return q_out;
}

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