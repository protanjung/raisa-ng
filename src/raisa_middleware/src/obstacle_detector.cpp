#include "pandu_ros2_kit/help_marker.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "raisa_interfaces/msg/obstacle_data.hpp"
#include "raisa_interfaces/msg/obstacle_parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#define LIMIT_X_MIN 0.00
#define LIMIT_X_MAX 1.50
#define LIMIT_Y_MIN -0.40
#define LIMIT_Y_MAX 0.40

#define EMERGENCY_X_MIN 0.00
#define EMERGENCY_X_MAX 0.50
#define EMERGENCY_Y_MIN -0.25
#define EMERGENCY_Y_MAX 0.25

using namespace std::chrono_literals;

class ObstacleDetector : public rclcpp::Node {
 public:
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_10hz;
  //-----Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_points;
  rclcpp::Subscription<raisa_interfaces::msg::ObstacleParameter>::SharedPtr sub_obstacle_parameter;
  //-----Pubisher
  rclcpp::Publisher<raisa_interfaces::msg::ObstacleData>::SharedPtr pub_obstacle_data;
  //-----Transform listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  //-----Help
  HelpMarker _marker;

  // Transform
  // =========
  bool tf_is_initialized = false;
  geometry_msgs::msg::TransformStamped tf_base_lidar;

  // Point cloud
  // ===========
  pcl::PointCloud<pcl::PointXYZ> points_lidar;
  pcl::PointCloud<pcl::PointXYZ> points_base;

  // Obstacle data and parameter
  // ===========================
  raisa_interfaces::msg::ObstacleData obstacle_data;
  raisa_interfaces::msg::ObstacleParameter obstacle_parameter;

  ObstacleDetector() : Node("obstacle_detector") {
    //-----Timer
    tim_10hz = this->create_wall_timer(100ms, std::bind(&ObstacleDetector::cllbck_tim_10hz, this));
    //-----Subscriber
    sub_lidar_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/points_xyz", 10, std::bind(&ObstacleDetector::cllbck_sub_lidar_points, this, std::placeholders::_1));
    sub_obstacle_parameter = this->create_subscription<raisa_interfaces::msg::ObstacleParameter>(
        "obstacle/parameter",
        10,
        std::bind(&ObstacleDetector::cllbck_sub_obstacle_parameter, this, std::placeholders::_1));
    //-----Pubisher
    pub_obstacle_data = this->create_publisher<raisa_interfaces::msg::ObstacleData>("obstacle/data", 10);
    //-----Tranform listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

    if (obstacle_detector_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "Obstacle detector init failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_10hz() {
    if (obstacle_detector_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "Obstacle detector routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_sub_lidar_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!tf_is_initialized) { return; }

    // =================================

    pcl::fromROSMsg(*msg, points_lidar);

    if (points_lidar.points.size() == 0) {
      obstacle_data.emergency = 0.85 * obstacle_data.emergency + 0.15 * 1.0;
      pub_obstacle_data->publish(obstacle_data);
      return;
    }

    pcl_ros::transformPointCloud(points_lidar, points_base, tf_base_lidar);

    // =================================

    static const float angle_start = -M_PI_2;
    static const float angle_stop = M_PI_2;
    static const float angle_step = (angle_stop - angle_start) / 32;

    // =================================

    float dst_max[32];
    float dst_zero[32], dst_one[32];
    float dst_raw[32], dst_normal[32];
    for (int i = 0; i < 32; i++) {
      dst_max[i] = obstacle_parameter.laser_scan_distance;
      if (dst_max[i] * sinf(angle_start + angle_step * (i + 0.5)) > LIMIT_Y_MAX) {
        dst_max[i] = LIMIT_Y_MAX / sinf(angle_start + angle_step * (i + 0.5));
      } else if (dst_max[i] * sinf(angle_start + angle_step * (i + 0.5)) < LIMIT_Y_MIN) {
        dst_max[i] = LIMIT_Y_MIN / sinf(angle_start + angle_step * (i + 0.5));
      }
      if (dst_max[i] * cosf(angle_start + angle_step * (i + 0.5)) > LIMIT_X_MAX) {
        dst_max[i] = LIMIT_X_MAX / cosf(angle_start + angle_step * (i + 0.5));
      } else if (dst_max[i] * cosf(angle_start + angle_step * (i + 0.5)) < LIMIT_X_MIN) {
        dst_max[i] = LIMIT_X_MIN / cosf(angle_start + angle_step * (i + 0.5));
      }

      dst_zero[i] = dst_max[i] * 0.5;
      dst_one[i] = dst_max[i] * 1.0;

      dst_raw[i] = dst_max[i];
      dst_normal[i] = 1.0;
    }

    // =================================

    for (auto p : points_base.points) {
      float dx = p.x - tf_base_lidar.transform.translation.x;
      float dy = p.y - tf_base_lidar.transform.translation.y;
      float r = sqrtf(dx * dx + dy * dy);
      float a = atan2f(dy, dx);

      int index = (a - angle_start) / angle_step;
      if (index < 0 || index >= 32) { continue; }

      if (r < dst_raw[index]) {
        dst_raw[index] = r;
        dst_normal[index] = fminf(1.0, fmaxf(0.0, (r - dst_zero[index]) / (dst_one[index] - dst_zero[index])));
      }
    }

    // =================================

    static std::vector<geometry_msgs::msg::Point> ps;
    static geometry_msgs::msg::Point p;

    ps.clear();
    for (int i = 0; i < 32; i++) {
      p.x = tf_base_lidar.transform.translation.x;
      p.y = tf_base_lidar.transform.translation.y;
      p.z = tf_base_lidar.transform.translation.z;
      ps.push_back(p);
      p.x += dst_raw[i] * cosf(angle_start + angle_step * (i + 0.5));
      p.y += dst_raw[i] * sinf(angle_start + angle_step * (i + 0.5));
      p.z += 0;
      ps.push_back(p);
    }
    _marker.line_list("base_link", "obstacle", 1, ps, std::vector<float>{0.87, 0.47, 0.34, 1}, 0.01);

    // =================================

    bool is_emergency = false;

    for (int i = 0; i < 32; i++) {
      float point_x = dst_raw[i] * cosf(angle_start + angle_step * (i + 0.5));
      float point_y = dst_raw[i] * sinf(angle_start + angle_step * (i + 0.5));
      if ((point_x > EMERGENCY_X_MIN && point_x < EMERGENCY_X_MAX) &&
          (point_y > EMERGENCY_Y_MIN && point_y < EMERGENCY_Y_MAX)) {
        is_emergency = true;
        break;
      }
    }

    if (is_emergency) {
      obstacle_data.emergency = 0.85 * obstacle_data.emergency + 0.15 * 1.0;
    } else {
      obstacle_data.emergency = 0.85 * obstacle_data.emergency + 0.15 * 0.0;
    }

    // =================================

    static const float k_steering_right[32] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                                               0.00, 0.00, 0.00, 0.00, 0.00, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50,
                                               0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.00, 0.00};
    static const float k_steering_left[32] = {0.00, 0.00, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.50,
                                              0.50, 0.50, 0.50, 0.50, 0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                                              0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
    static const float k_velocity_right[32] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                                               0.00, 0.00, 0.00, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50,
                                               0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.00, 0.00};
    static const float k_velocity_left[32] = {0.00, 0.00, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.50,
                                              0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.00, 0.00, 0.00, 0.00,
                                              0.00, 0.0,  0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
    static const float k_velocity_following[32] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                                                   0.00, 0.25, 0.25, 0.50, 0.50, 0.50, 0.50, 0.25, 0.25, 0.00, 0.00,
                                                   0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

    float num_steering_right = 0.0, den_steering_right = 0.0;
    float num_steering_left = 0.0, den_steering_left = 0.0;
    float num_velocity_right = 0.0, den_velocity_right = 0.0;
    float num_velocity_left = 0.0, den_velocity_left = 0.0;
    float num_velocity_following = 0.0, den_velocity_following = 0.0;

    for (int i = 0; i < 32; i++) {
      num_steering_right += k_steering_right[i] * (1 - dst_normal[i]);
      den_steering_right += k_steering_right[i];
      num_steering_left += k_steering_left[i] * (1 - dst_normal[i]);
      den_steering_left += k_steering_left[i];
      num_velocity_right += k_velocity_right[i] * (1 - dst_normal[i]);
      den_velocity_right += k_velocity_right[i];
      num_velocity_left += k_velocity_left[i] * (1 - dst_normal[i]);
      den_velocity_left += k_velocity_left[i];
      num_velocity_following += k_velocity_following[i] * (1 - dst_normal[i]);
      den_velocity_following += k_velocity_following[i];
    }

    obstacle_data.steering_right =
        0.85 * obstacle_data.steering_right + 0.15 * (num_steering_right / den_steering_right);
    obstacle_data.steering_left = 0.85 * obstacle_data.steering_left + 0.15 * (num_steering_left / den_steering_left);
    obstacle_data.velocity_right =
        0.85 * obstacle_data.velocity_right + 0.15 * (num_velocity_right / den_velocity_right);
    obstacle_data.velocity_left = 0.85 * obstacle_data.velocity_left + 0.15 * (num_velocity_left / den_velocity_left);
    obstacle_data.velocity_following =
        0.85 * obstacle_data.velocity_following + 0.15 * (num_velocity_following / den_velocity_following);

    if (obstacle_parameter.status_steering == false) {
      obstacle_data.steering_right = 0.0;
      obstacle_data.steering_left = 0.0;
    }

    if (obstacle_parameter.status_velocity == false) {
      obstacle_data.velocity_right = 0.0;
      obstacle_data.velocity_left = 0.0;
      obstacle_data.velocity_following = 0.0;
    }

    // =================================

    pub_obstacle_data->publish(obstacle_data);
  }

  void cllbck_sub_obstacle_parameter(const raisa_interfaces::msg::ObstacleParameter::SharedPtr msg) {
    obstacle_parameter = *msg;
  }

  //====================================

  bool obstacle_detector_init() {
    while (!tf_is_initialized) {
      try {
        tf_base_lidar = tf_buffer->lookupTransform("base_link", "lidar_link", tf2::TimePointZero);
        tf_is_initialized = true;
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        RCLCPP_WARN(this->get_logger(), "If this warning appears for a long time, please check the tf tree");
        rclcpp::sleep_for(1s);
      }
    }

    return true;
  }

  bool obstacle_detector_routine() {
    if (!_marker.is_initialized()) { _marker.init(this->shared_from_this()); }

    return true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_obstacle_detector = std::make_shared<ObstacleDetector>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_obstacle_detector);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}