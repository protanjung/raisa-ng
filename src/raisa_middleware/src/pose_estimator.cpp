#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "raisa_interfaces/msg/stm32_to_pc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define RADIUS(b, a) sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2))
#define ANGLE(b, a) atan2(b.y - a.y, b.x - a.x)

using namespace std::chrono_literals;

class PoseEstimator : public rclcpp::Node {
 public:
  //-----Parameter
  double raisa_encoder_odometry_pulse_to_meter;
  double raisa_roda_pulse_roda_to_meter;
  double raisa_odometry_offset_x;
  double raisa_odometry_offset_y;
  double raisa_roda_offset_x;
  double raisa_roda_offset_y;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_100hz;
  //-----Subscriber
  rclcpp::Subscription<raisa_interfaces::msg::Stm32ToPc>::SharedPtr sub_stm32_to_pc;
  //-----Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

  // Encoder dan gyroscope
  // =====================
  uint16_t encoder_odometry0;
  uint16_t encoder_odometry1;
  uint16_t encoder_roda0;
  uint16_t encoder_roda1;
  uint16_t encoder_roda2;
  float gyroscope;

  // Pose and twist
  // ==============
  geometry_msgs::msg::Twist odometry_twist;
  geometry_msgs::msg::Pose2D odometry_pose2d;
  geometry_msgs::msg::Twist odometry_twist_aux;
  geometry_msgs::msg::Pose2D odometry_pose2d_aux;

  PoseEstimator() : Node("pose_estimator") {
    //-----Parameter
    this->declare_parameter("raisa.conversion.odometry_pulse_to_meter", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("raisa.conversion.roda_pulse_to_meter", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("raisa.odometry.offset_x", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("raisa.odometry.offset_y", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("raisa.roda.offset_x", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("raisa.roda.offset_y", rclcpp::PARAMETER_DOUBLE);
    this->get_parameter("raisa.conversion.odometry_pulse_to_meter", raisa_encoder_odometry_pulse_to_meter);
    this->get_parameter("raisa.conversion.roda_pulse_to_meter", raisa_roda_pulse_roda_to_meter);
    this->get_parameter("raisa.odometry.offset_x", raisa_odometry_offset_x);
    this->get_parameter("raisa.odometry.offset_y", raisa_odometry_offset_y);
    this->get_parameter("raisa.roda.offset_x", raisa_roda_offset_x);
    this->get_parameter("raisa.roda.offset_y", raisa_roda_offset_y);
    //-----Timer
    tim_100hz = this->create_wall_timer(10ms, std::bind(&PoseEstimator::cllbck_tim_100hz, this));
    //-----Subscriber
    sub_stm32_to_pc = this->create_subscription<raisa_interfaces::msg::Stm32ToPc>(
        "stm32/to_pc", 10, std::bind(&PoseEstimator::cllbck_sub_stm32_to_pc, this, std::placeholders::_1));
    //-----Publisher
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    if (pose_estimator_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "Pose estimator init failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_100hz() {
    if (pose_estimator_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "Pose estimator routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_sub_stm32_to_pc(const raisa_interfaces::msg::Stm32ToPc::SharedPtr msg) {
    encoder_odometry0 = msg->encoder_odometry0;
    encoder_odometry1 = msg->encoder_odometry1;
    encoder_roda0 = msg->encoder_roda0;
    encoder_roda1 = msg->encoder_roda1;
    encoder_roda2 = msg->encoder_roda2;
    gyroscope = msg->gyroscope;

    // =================================

    static rclcpp::Time last_time = this->now();
    static rclcpp::Time current_time = this->now();
    last_time = current_time;
    current_time = this->now();
    double dt = (current_time - last_time).seconds();

    // =================================

    int16_t d_encoder0 = 0, d_encoder1 = 0;
    delta_encoder(encoder_odometry0, encoder_odometry1, d_encoder0, d_encoder1);
    if (abs(d_encoder0) > 16384 || abs(d_encoder1) > 16384) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Delta encoder value is too big (%d, %d). Possible of connection error.",
          d_encoder0,
          d_encoder1);
      return;
    }

    int16_t d_encoder0_aux = 0, d_encoder1_aux = 0, d_encoder2_aux = 0;
    delta_encoder_aux(encoder_roda0, encoder_roda1, encoder_roda2, d_encoder0_aux, d_encoder1_aux, d_encoder2_aux);
    if (abs(d_encoder0_aux) > 2048 || abs(d_encoder1_aux) > 2048 || abs(d_encoder2_aux) > 2048) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Delta encoder aux value is too big (%d, %d, %d). Possible of connection error.",
          d_encoder0_aux,
          d_encoder1_aux,
          d_encoder2_aux);
      return;
    }

    float d_gyroscope = 0;
    delta_gyroscope(gyroscope, d_gyroscope);
    if (abs(d_gyroscope) > 90) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Delta gyroscope sensor value is too big (%f). Possible of connection error.",
          d_gyroscope);
      return;
    }

    // =================================

    static geometry_msgs::msg::Pose2D temp_pose2d;

    static bool first_time = true;
    if (first_time == true) {
      temp_pose2d.x = -raisa_odometry_offset_x;
      temp_pose2d.y = -raisa_odometry_offset_y;
      temp_pose2d.theta = 0;
      first_time = false;
    }

    temp_pose2d.x +=
        (d_encoder0 * cosf(temp_pose2d.theta + 0.7853981634) + d_encoder1 * cosf(temp_pose2d.theta - 0.7853981634)) *
        raisa_encoder_odometry_pulse_to_meter;
    temp_pose2d.y +=
        (d_encoder0 * sinf(temp_pose2d.theta + 0.7853981634) + d_encoder1 * sinf(temp_pose2d.theta - 0.7853981634)) *
        raisa_encoder_odometry_pulse_to_meter;

    temp_pose2d.theta += d_gyroscope * M_PI / 180;
    if (temp_pose2d.theta > M_PI) {
      temp_pose2d.theta -= 2 * M_PI;
    } else if (temp_pose2d.theta < -M_PI) {
      temp_pose2d.theta += 2 * M_PI;
    }

    // =================================

    static geometry_msgs::msg::Pose2D temp_pose2d0;
    static geometry_msgs::msg::Pose2D temp_pose2d1;
    temp_pose2d0 = temp_pose2d1;

    double offset_r = sqrt(pow(raisa_odometry_offset_x, 2) + pow(raisa_odometry_offset_y, 2));
    double offset_a = atan2(raisa_odometry_offset_y, raisa_odometry_offset_x);
    temp_pose2d1.x = temp_pose2d.x + offset_r * cosf(temp_pose2d.theta + offset_a);
    temp_pose2d1.y = temp_pose2d.y + offset_r * sinf(temp_pose2d.theta + offset_a);
    temp_pose2d1.theta = temp_pose2d.theta;

    // =================================

    odometry_pose2d.x = temp_pose2d1.x;
    odometry_pose2d.y = temp_pose2d1.y;
    odometry_pose2d.theta = temp_pose2d1.theta;

    double twist_r = sqrt(pow(temp_pose2d1.x - temp_pose2d0.x, 2) + pow(temp_pose2d1.y - temp_pose2d0.y, 2));
    double twist_a = atan2(temp_pose2d1.y - temp_pose2d0.y, temp_pose2d1.x - temp_pose2d0.x) - temp_pose2d0.theta;
    odometry_twist.linear.x = twist_r * cosf(twist_a) / dt;
    odometry_twist.linear.y = twist_r * sinf(twist_a) / dt;
    odometry_twist.angular.z = d_gyroscope * M_PI / 180 / dt;

    // =================================

    nav_msgs::msg::Odometry msg_odom;
    msg_odom.header.stamp = current_time;
    msg_odom.header.frame_id = "odom";
    msg_odom.child_frame_id = "base_link";
    msg_odom.pose.pose.position.x = odometry_pose2d.x;
    msg_odom.pose.pose.position.y = odometry_pose2d.y;
    msg_odom.pose.pose.position.z = 0;
    msg_odom.pose.pose.orientation = rpy_to_quaternion(0, 0, odometry_pose2d.theta);
    msg_odom.pose.covariance[0] = 1e-12;
    msg_odom.pose.covariance[7] = 1e-12;
    msg_odom.pose.covariance[14] = 1e6;
    msg_odom.pose.covariance[21] = 1e6;
    msg_odom.pose.covariance[28] = 1e6;
    msg_odom.pose.covariance[35] = 1e-12;
    msg_odom.twist.twist = odometry_twist;
    msg_odom.twist.covariance[0] = 1e-12;
    msg_odom.twist.covariance[7] = 1e-12;
    msg_odom.twist.covariance[14] = 1e6;
    msg_odom.twist.covariance[21] = 1e6;
    msg_odom.twist.covariance[28] = 1e6;
    msg_odom.twist.covariance[35] = 1e-12;
    pub_odom->publish(msg_odom);

    // =================================

    static geometry_msgs::msg::Pose2D temp_pose2d_aux;

    static bool first_time_aux = true;
    if (first_time_aux == true) {
      temp_pose2d_aux.x = -raisa_roda_offset_x;
      temp_pose2d_aux.y = -raisa_roda_offset_y;
      temp_pose2d_aux.theta = 0;
      first_time_aux = false;
    }

    temp_pose2d_aux.x += (d_encoder0_aux * cosf(temp_pose2d_aux.theta + 2.6179938780) +
                          d_encoder1_aux * cosf(temp_pose2d_aux.theta + 0.5235987756) +
                          d_encoder2_aux * cosf(temp_pose2d_aux.theta - 1.5707963268)) *
                         raisa_roda_pulse_roda_to_meter;
    temp_pose2d_aux.y += (d_encoder0_aux * sinf(temp_pose2d_aux.theta + 2.6179938780) +
                          d_encoder1_aux * sinf(temp_pose2d_aux.theta + 0.5235987756) +
                          d_encoder2_aux * sinf(temp_pose2d_aux.theta - 1.5707963268)) *
                         raisa_roda_pulse_roda_to_meter;

    temp_pose2d_aux.theta += d_gyroscope * M_PI / 180;
    if (temp_pose2d_aux.theta > M_PI) {
      temp_pose2d_aux.theta -= 2 * M_PI;
    } else if (temp_pose2d_aux.theta < -M_PI) {
      temp_pose2d_aux.theta += 2 * M_PI;
    }

    // =================================

    static geometry_msgs::msg::Pose2D temp_pose2d0_aux;
    static geometry_msgs::msg::Pose2D temp_pose2d1_aux;
    temp_pose2d0_aux = temp_pose2d1_aux;

    double offset_r_aux = sqrt(pow(raisa_roda_offset_x, 2) + pow(raisa_roda_offset_y, 2));
    double offset_a_aux = atan2(raisa_roda_offset_y, raisa_roda_offset_x);
    temp_pose2d1_aux.x = temp_pose2d_aux.x + offset_r_aux * cosf(temp_pose2d_aux.theta + offset_a_aux);
    temp_pose2d1_aux.y = temp_pose2d_aux.y + offset_r_aux * sinf(temp_pose2d_aux.theta + offset_a_aux);
    temp_pose2d1_aux.theta = temp_pose2d.theta;

    // =================================

    odometry_pose2d_aux.x = temp_pose2d1_aux.x;
    odometry_pose2d_aux.y = temp_pose2d1_aux.y;
    odometry_pose2d_aux.theta = temp_pose2d1_aux.theta;

    double twist_r_aux =
        sqrt(pow(temp_pose2d1_aux.x - temp_pose2d0_aux.x, 2) + pow(temp_pose2d1_aux.y - temp_pose2d0_aux.y, 2));
    double twist_a_aux = atan2(temp_pose2d1_aux.y - temp_pose2d0_aux.y, temp_pose2d1_aux.x - temp_pose2d0_aux.x) -
                         temp_pose2d0_aux.theta;
    odometry_twist_aux.linear.x = twist_r_aux * cosf(twist_a_aux) / dt;
    odometry_twist_aux.linear.y = twist_r_aux * sinf(twist_a_aux) / dt;
    odometry_twist_aux.angular.z = d_gyroscope * M_PI / 180 / dt;
  }

  //====================================

  bool pose_estimator_init() {
    RCLCPP_INFO(this->get_logger(), "Odometry Pulse to Meter: %f", raisa_encoder_odometry_pulse_to_meter);
    RCLCPP_INFO(this->get_logger(), "Roda Pulse to Meter: %f", raisa_roda_pulse_roda_to_meter);
    RCLCPP_INFO(this->get_logger(), "Odometry Offset X: %f", raisa_odometry_offset_x);
    RCLCPP_INFO(this->get_logger(), "Odometry Offset Y: %f", raisa_odometry_offset_y);
    RCLCPP_INFO(this->get_logger(), "Roda Offset X: %f", raisa_roda_offset_x);
    RCLCPP_INFO(this->get_logger(), "Roda Offset Y: %f", raisa_roda_offset_y);

    return true;
  }

  bool pose_estimator_routine() { return true; }

  //====================================

  void delta_encoder(uint16_t encoder0, uint16_t encoder1, int16_t &d_encoder0, int16_t &d_encoder1) {
    static bool is_initialized = false;
    static uint16_t last_encoder0 = 0;
    static uint16_t last_encoder1 = 0;

    if (is_initialized == false && (last_encoder0 == 0 || last_encoder1 == 0)) {
      last_encoder0 = encoder0;
      last_encoder1 = encoder1;
      is_initialized = true;
      return;
    }

    if (encoder0 < 16384 && last_encoder0 > 49152) {
      d_encoder0 = encoder0 - last_encoder0 + 65536;
    } else if (encoder0 > 49152 && last_encoder0 < 16384) {
      d_encoder0 = encoder0 - last_encoder0 - 65536;
    } else {
      d_encoder0 = encoder0 - last_encoder0;
    }

    if (encoder1 < 16384 && last_encoder1 > 49152) {
      d_encoder1 = encoder1 - last_encoder1 + 65536;
    } else if (encoder1 > 49152 && last_encoder1 < 16384) {
      d_encoder1 = encoder1 - last_encoder1 - 65536;
    } else {
      d_encoder1 = encoder1 - last_encoder1;
    }

    last_encoder0 = encoder0;
    last_encoder1 = encoder1;
  }

  void delta_encoder_aux(
      uint16_t encoder0,
      uint16_t encoder1,
      uint16_t encoder2,
      int16_t &d_encoder0,
      int16_t &d_encoder1,
      int16_t &d_encoder2) {
    static bool is_initialized = false;
    static uint16_t last_encoder0 = 0;
    static uint16_t last_encoder1 = 0;
    static uint16_t last_encoder2 = 0;

    if (is_initialized == false && (last_encoder0 == 0 || last_encoder1 == 0 || last_encoder2 == 0)) {
      last_encoder0 = encoder0;
      last_encoder1 = encoder1;
      last_encoder2 = encoder2;
      is_initialized = true;
      return;
    }

    if (encoder0 < 1024 && last_encoder0 > 3072) {
      d_encoder0 = encoder0 - last_encoder0 + 4096;
    } else if (encoder0 > 3072 && last_encoder0 < 1024) {
      d_encoder0 = encoder0 - last_encoder0 - 4096;
    } else {
      d_encoder0 = encoder0 - last_encoder0;
    }

    if (encoder1 < 1024 && last_encoder1 > 3072) {
      d_encoder1 = encoder1 - last_encoder1 + 4096;
    } else if (encoder1 > 3072 && last_encoder1 < 1024) {
      d_encoder1 = encoder1 - last_encoder1 - 4096;
    } else {
      d_encoder1 = encoder1 - last_encoder1;
    }

    if (encoder2 < 1024 && last_encoder2 > 3072) {
      d_encoder2 = encoder2 - last_encoder2 + 4096;
    } else if (encoder2 > 3072 && last_encoder2 < 1024) {
      d_encoder2 = encoder2 - last_encoder2 - 4096;
    } else {
      d_encoder2 = encoder2 - last_encoder2;
    }

    last_encoder0 = encoder0;
    last_encoder1 = encoder1;
    last_encoder2 = encoder2;
  }

  void delta_gyroscope(float gyroscope, float &d_gyroscope) {
    static bool is_initialized = false;
    static float last_gyroscope = 0;

    if (is_initialized == false && last_gyroscope == 0) {
      last_gyroscope = gyroscope;
      is_initialized = true;
      return;
    }

    if (gyroscope < -90 && last_gyroscope > 90) {
      d_gyroscope = gyroscope - last_gyroscope + 360;
    } else if (gyroscope > 90 && last_gyroscope < -90) {
      d_gyroscope = gyroscope - last_gyroscope - 360;
    } else {
      d_gyroscope = gyroscope - last_gyroscope;
    }

    last_gyroscope = gyroscope;
  }

  //====================================

  geometry_msgs::msg::Quaternion rpy_to_quaternion(float roll, float pitch, float yaw) {
    tf2::Quaternion q_in;
    geometry_msgs::msg::Quaternion q_out;

    q_in.setRPY(roll, pitch, yaw);
    tf2::convert(q_in, q_out);

    return q_out;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_pose_estimator = std::make_shared<PoseEstimator>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_pose_estimator);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}