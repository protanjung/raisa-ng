#include "tf2_ros/transform_broadcaster.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class TransformBroadcaster : public rclcpp::Node {
 public:
  //-----Parameter
  std::vector<double> raisa_tf_camera;
  std::vector<double> raisa_tf_lidar;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_100hz;
  //-----Transform broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

  TransformBroadcaster() : Node("transform_broadcaster") {
    //-----Parameter
    this->declare_parameter("raisa.tf.camera", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("raisa.tf.lidar", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->get_parameter("raisa.tf.camera", raisa_tf_camera);
    this->get_parameter("raisa.tf.lidar", raisa_tf_lidar);

    //-----Timer
    tim_100hz = this->create_wall_timer(10ms, std::bind(&TransformBroadcaster::cllbck_tim_100hz, this));
    //-----Transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    if (transform_broadcaster_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "Transform broadcaster init failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_100hz() {
    if (transform_broadcaster_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "Transform broadcaster routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  bool transform_broadcaster_init() {
    send_static_transform(raisa_tf_camera, "base_link", "camera_link");
    send_static_transform(raisa_tf_lidar, "base_link", "lidar_link");

    return true;
  }

  bool transform_broadcaster_routine() { return true; }

  //====================================

  void send_transform(std::vector<double> tf, std::string frame_id, std::string child_frame_id) {
    if (tf.size() != 6) { return; }

    tf2::Vector3 origin;
    origin.setValue(tf[0], tf[1], tf[2]);

    tf2::Quaternion rotation;
    rotation.setRPY(tf[3] * M_PI / 180, tf[4] * M_PI / 180, tf[5] * M_PI / 180);

    geometry_msgs::msg::TransformStamped transform_stamped;
    // ----
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    // ----
    transform_stamped.transform.translation.x = origin.x();
    transform_stamped.transform.translation.y = origin.y();
    transform_stamped.transform.translation.z = origin.z();
    // ----
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();
    // ----
    tf_broadcaster->sendTransform(transform_stamped);
  }

  void send_static_transform(std::vector<double> tf, std::string frame_id, std::string child_frame_id) {
    if (tf.size() != 6) { return; }

    tf2::Vector3 origin;
    origin.setValue(tf[0], tf[1], tf[2]);

    tf2::Quaternion rotation;
    rotation.setRPY(tf[3] * M_PI / 180, tf[4] * M_PI / 180, tf[5] * M_PI / 180);

    geometry_msgs::msg::TransformStamped transform_stamped;
    // ----
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    // ----
    transform_stamped.transform.translation.x = origin.x();
    transform_stamped.transform.translation.y = origin.y();
    transform_stamped.transform.translation.z = origin.z();
    // ----
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();
    // ----
    static_tf_broadcaster->sendTransform(transform_stamped);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_transform_broadcaster = std::make_shared<TransformBroadcaster>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_transform_broadcaster);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}