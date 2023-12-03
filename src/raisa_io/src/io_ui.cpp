#include "pandu_ros2_kit/udp.hpp"
#include "raisa_interfaces/msg/ui_from_pc.hpp"
#include "raisa_interfaces/msg/ui_to_pc.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class IOUI : public rclcpp::Node {
 public:
  //-----Parameter
  int ui_port;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_1000hz;
  //-----Subscriber
  rclcpp::Subscription<raisa_interfaces::msg::UiFromPc>::SharedPtr sub_ui_from_pc;
  //-----Publisher
  rclcpp::Publisher<raisa_interfaces::msg::UiToPc>::SharedPtr pub_ui_to_pc;

  // Socket connection
  // =================
  UDP ui_udp;
  uint8_t ui_tx_buffer[2048];
  uint8_t ui_rx_buffer[2048];

  // Data to PC
  // ==========

  // Data from PC
  // ============
  uint8_t state_machine;
  uint8_t human_presence;
  uint8_t human_temperature;

  IOUI() : Node("io_ui") {
    //-----Parameter
    this->declare_parameter("ui.port", rclcpp::PARAMETER_INTEGER);
    this->get_parameter("ui.port", ui_port);
    //-----Timer
    tim_1000hz = this->create_wall_timer(1ms, std::bind(&IOUI::cllbck_tim_1000hz, this));
    //-----Subscriber
    sub_ui_from_pc = this->create_subscription<raisa_interfaces::msg::UiFromPc>(
        "ui/from_pc", 10, std::bind(&IOUI::cllbck_sub_ui_from_pc, this, std::placeholders::_1));
    //-----Publisher
    pub_ui_to_pc = this->create_publisher<raisa_interfaces::msg::UiToPc>("ui/to_pc", 10);

    if (ui_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "UI init failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_1000hz() {
    if (ui_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "UI routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_sub_ui_from_pc(const raisa_interfaces::msg::UiFromPc::SharedPtr msg) {
    state_machine = msg->state_machine;
    human_presence = msg->human_presence;
    human_temperature = msg->human_temperature;
  }

  //====================================

  bool ui_init() {
    RCLCPP_INFO(this->get_logger(), "Port: %d", ui_port);

    if (ui_udp.init_as_server(ui_port) == false) {
      RCLCPP_ERROR(this->get_logger(), "UDP init failed");
      return false;
    }

    return true;
  }

  bool ui_routine() {
    int len_data_to_pc = -1;
    int len_data_from_pc = -1;

    if ((len_data_to_pc = ui_udp.recv(ui_rx_buffer, sizeof(ui_rx_buffer))) >= 0) {
      // Communication Protocol (UI -> PC)
      // =================================
      // Offset   | Size  | Description

      // Communication Protocol (PC -> UI)
      // =================================
      // Offset   | Size  | Description
      // 0        | 1     | state_machine
      // 1        | 1     | human_presence
      // 2        | 1     | human_temperature
      memcpy(ui_tx_buffer + 0, &state_machine, 1);
      memcpy(ui_tx_buffer + 1, &human_presence, 1);
      memcpy(ui_tx_buffer + 2, &human_temperature, 1);

      len_data_from_pc = ui_udp.send(ui_tx_buffer, 3);
    }

    // =================================

    static rclcpp::Time time_old = this->now();

    if (len_data_from_pc >= 0) { time_old = this->now(); }

    if (this->now() - time_old > 1s) {}

    // =================================

    static raisa_interfaces::msg::UiToPc msg_ui_to_pc;
    // ----
    // ----
    pub_ui_to_pc->publish(msg_ui_to_pc);

    return true;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node_io_ui = std::make_shared<IOUI>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_io_ui);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}