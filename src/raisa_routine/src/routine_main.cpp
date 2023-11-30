#include "raisa_routine/routine.hpp"

using namespace std::chrono_literals;

Routine::Routine() : Node("routine") {
  //-----Parameter
  this->declare_parameter("raisa.conversion.stm32_from_pc_linear_multiplier", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raisa.conversion.stm32_from_pc_angular_multiplier", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raisa.roda.offset_x", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raisa.roda.offset_y", rclcpp::PARAMETER_DOUBLE);
  this->get_parameter(
      "raisa.conversion.stm32_from_pc_linear_multiplier", raisa_conversion_stm32_from_pc_linear_multiplier);
  this->get_parameter(
      "raisa.conversion.stm32_from_pc_angular_multiplier", raisa_conversion_stm32_from_pc_angular_multiplier);
  this->get_parameter("raisa.roda.offset_x", raisa_roda_offset_x);
  this->get_parameter("raisa.roda.offset_y", raisa_roda_offset_y);
  //-----Timer
  tim_10hz = this->create_wall_timer(100ms, std::bind(&Routine::cllbck_tim_10hz, this));
  //-----Subscriber
  sub_stm32_to_pc = this->create_subscription<raisa_interfaces::msg::Stm32ToPc>(
      "stm32/to_pc", 10, std::bind(&Routine::cllbck_sub_stm32_to_pc, this, std::placeholders::_1));
  sub_basestation_to_pc = this->create_subscription<raisa_interfaces::msg::BasestationToPc>(
      "basestation/to_pc", 10, std::bind(&Routine::cllbck_sub_basestaion_to_pc, this, std::placeholders::_1));
  //-----Publisher
  pub_stm32_from_pc = this->create_publisher<raisa_interfaces::msg::Stm32FromPc>("stm32/from_pc", 10);
  pub_basestation_from_pc = this->create_publisher<raisa_interfaces::msg::BasestationFromPc>("basestation/from_pc", 10);
}

//====================================

void Routine::cllbck_tim_10hz() {}

//====================================

void Routine::cllbck_sub_stm32_to_pc(raisa_interfaces::msg::Stm32ToPc::SharedPtr msg) { (void)msg; }

void Routine::cllbck_sub_basestaion_to_pc(raisa_interfaces::msg::BasestationToPc::SharedPtr msg) { (void)msg; }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node_routine = std::make_shared<Routine>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_routine);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
