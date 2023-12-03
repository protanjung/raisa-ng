#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class IOVision : public rclcpp::Node {
 public:
  //-----Parameter
  std::string camera_path;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_50hz;
  //-----Publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_bgr;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_gray;

  // Image processing
  // ================
  cv::VideoCapture cap;
  cv::Mat frame_bgr, frame_gray;

  IOVision() : Node("io_vision") {
    //-----Parameter
    this->declare_parameter("camera.path", rclcpp::PARAMETER_STRING);
    this->get_parameter("camera.path", camera_path);
    //-----Timer
    tim_50hz = this->create_wall_timer(20ms, std::bind(&IOVision::cllbck_tim_50hz, this));
    //-----Publisher
    pub_image_bgr = this->create_publisher<sensor_msgs::msg::Image>("image_bgr", 1);
    pub_image_gray = this->create_publisher<sensor_msgs::msg::Image>("image_gray", 1);

    if (io_vision_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "Vision init failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_50hz() {
    if (io_vision_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "Vision routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  bool io_vision_init() {
    RCLCPP_INFO(this->get_logger(), "Path: %s", camera_path.c_str());

    if (!cap.open(camera_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
      return false;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FPS, 30);

    return true;
  }

  bool io_vision_routine() {
    if (!cap.read(frame_bgr)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read camera");
      return false;
    }

    cv::cvtColor(frame_bgr, frame_gray, cv::COLOR_BGR2GRAY);

    auto msg_image_bgr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_bgr).toImageMsg();
    auto msg_image_gray = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", frame_gray).toImageMsg();
    pub_image_bgr->publish(*msg_image_bgr);
    pub_image_gray->publish(*msg_image_gray);

    return true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_io_vision = std::make_shared<IOVision>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_io_vision);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}