#ifndef ROUTINE_HPP_
#define ROUTINE_HPP_

#include "raisa_interfaces/msg/basestation_from_pc.hpp"
#include "raisa_interfaces/msg/basestation_to_pc.hpp"
#include "raisa_interfaces/msg/stm32_from_pc.hpp"
#include "raisa_interfaces/msg/stm32_to_pc.hpp"
#include "rclcpp/rclcpp.hpp"

class Routine : public rclcpp::Node {
 public:
  //-----Parameter
  double raisa_conversion_stm32_from_pc_linear_multiplier;
  double raisa_conversion_stm32_from_pc_angular_multiplier;
  double raisa_roda_offset_x;
  double raisa_roda_offset_y;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_10hz;
  //-----Subscriber
  rclcpp::Subscription<raisa_interfaces::msg::Stm32ToPc>::SharedPtr sub_stm32_to_pc;
  rclcpp::Subscription<raisa_interfaces::msg::BasestationToPc>::SharedPtr sub_basestation_to_pc;
  //-----Publisher
  rclcpp::Publisher<raisa_interfaces::msg::Stm32FromPc>::SharedPtr pub_stm32_from_pc;
  rclcpp::Publisher<raisa_interfaces::msg::BasestationFromPc>::SharedPtr pub_basestation_from_pc;

  Routine();

  void cllbck_tim_10hz();

  void cllbck_sub_stm32_to_pc(raisa_interfaces::msg::Stm32ToPc::SharedPtr msg);
  void cllbck_sub_basestaion_to_pc(raisa_interfaces::msg::BasestationToPc::SharedPtr msg);
};

#endif