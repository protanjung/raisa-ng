#include "pandu_ros2_kit/udp.hpp"
#include "raisa_interfaces/msg/stm32_from_pc.hpp"
#include "raisa_interfaces/msg/stm32_to_pc.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class IOSTM32 : public rclcpp::Node {
 public:
  //-----Parameter
  std::string stm32_ip;
  int stm32_port;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_30hz;
  //-----Subscriber
  rclcpp::Subscription<raisa_interfaces::msg::Stm32FromPc>::SharedPtr sub_stm32_from_pc;
  //-----Publisher
  rclcpp::Publisher<raisa_interfaces::msg::Stm32ToPc>::SharedPtr pub_stm32_to_pc;

  // Socket connection
  // =================
  UDP stm32_udp;
  uint8_t stm32_tx_buffer[2048];
  uint8_t stm32_rx_buffer[2048];

  // Data to PC
  // ==========
  uint32_t epoch_to_pc;
  uint16_t encoder_roda0;
  uint16_t encoder_roda1;
  uint16_t encoder_roda2;
  uint16_t encoder_roda3;
  float gyroscope;
  float battery_voltage;
  float battery_soc;

  // Data from PC
  // ============
  uint32_t epoch_from_pc;
  uint8_t mode;
  int16_t dx;
  int16_t dy;
  int16_t dtheta;

  IOSTM32() : Node("io_stm32") {
    //-----Parameter
    this->declare_parameter("stm32.ip", rclcpp::PARAMETER_STRING);
    this->declare_parameter("stm32.port", rclcpp::PARAMETER_INTEGER);
    this->get_parameter("stm32.ip", stm32_ip);
    this->get_parameter("stm32.port", stm32_port);
    //-----Timer
    tim_30hz = this->create_wall_timer(33ms, std::bind(&IOSTM32::cllbck_tim_30hz, this));
    //-----Subscriber
    sub_stm32_from_pc = this->create_subscription<raisa_interfaces::msg::Stm32FromPc>(
        "stm32/from_pc", 10, std::bind(&IOSTM32::cllbck_sub_stm32_from_pc, this, std::placeholders::_1));
    //-----Publisher
    pub_stm32_to_pc = this->create_publisher<raisa_interfaces::msg::Stm32ToPc>("stm32/to_pc", 10);

    if (stm32_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "STM32 init failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_30hz() {
    if (stm32_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "STM32 routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_sub_stm32_from_pc(const raisa_interfaces::msg::Stm32FromPc::SharedPtr msg) {
    mode = msg->mode;
    dx = msg->dx;
    dy = msg->dy;
    dtheta = msg->dtheta;
  }

  //====================================

  bool stm32_init() {
    RCLCPP_INFO(this->get_logger(), "IP: %s", stm32_ip.c_str());
    RCLCPP_INFO(this->get_logger(), "Port: %d", stm32_port);

    if (stm32_udp.init_as_client(stm32_ip, stm32_port) == false) {
      RCLCPP_ERROR(this->get_logger(), "STM32 UDP init failed");
      return false;
    }

    return true;
  }

  bool stm32_routine() {
    // Communication Protocol (PC -> STM32)
    // ====================================
    // Offset   | Size  | Description
    // 0        | 4     | epoch_from_pc
    // 4        | 1     | mode
    // 5        | 2     | dx
    // 7        | 2     | dy
    // 9        | 2     | dtheta
    memcpy(stm32_tx_buffer + 0, &epoch_from_pc, 4);
    memcpy(stm32_tx_buffer + 4, &mode, 1);
    memcpy(stm32_tx_buffer + 5, &dx, 2);
    memcpy(stm32_tx_buffer + 7, &dy, 2);
    memcpy(stm32_tx_buffer + 9, &dtheta, 2);

    int len_data_from_pc = stm32_udp.send(stm32_tx_buffer, 12);
    int len_data_to_pc = stm32_udp.recv(stm32_rx_buffer, 24);

    // Communication Protocol (STM32 -> PC)
    // ====================================
    // Offset   | Size  | Description
    // 0        | 4     | epoch_to_pc
    // 4        | 2     | encoder_roda0
    // 6        | 2     | encoder_roda1
    // 8        | 2     | encoder_roda2
    // 10       | 2     | encoder_roda3
    // 12       | 4     | gyroscope
    // 16       | 4     | battery_voltage
    // 20       | 4     | battery_soc

    memcpy(&epoch_to_pc, stm32_rx_buffer + 0, 4);
    memcpy(&encoder_roda0, stm32_rx_buffer + 4, 2);
    memcpy(&encoder_roda1, stm32_rx_buffer + 6, 2);
    memcpy(&encoder_roda2, stm32_rx_buffer + 8, 2);
    memcpy(&encoder_roda3, stm32_rx_buffer + 10, 2);
    memcpy(&gyroscope, stm32_rx_buffer + 12, 4);
    memcpy(&battery_voltage, stm32_rx_buffer + 16, 4);
    memcpy(&battery_soc, stm32_rx_buffer + 20, 4);

    // =================================

    if (len_data_from_pc < 0) { return true; }
    if (len_data_to_pc < 0) { return true; }

    // =================================

    epoch_from_pc++;

    // =================================

    static uint32_t last_epoch_to_pc = 0;

    if (last_epoch_to_pc == 0) {
      last_epoch_to_pc = epoch_to_pc;
      return true;
    }

    if (epoch_to_pc != last_epoch_to_pc + 1) {
      last_epoch_to_pc = epoch_to_pc;
      return true;
    }

    last_epoch_to_pc = epoch_to_pc;

    // =================================

    static raisa_interfaces::msg::Stm32ToPc msg_stm32_to_pc;
    // ----
    msg_stm32_to_pc.encoder_roda0 = encoder_roda0;
    msg_stm32_to_pc.encoder_roda1 = encoder_roda1;
    msg_stm32_to_pc.encoder_roda2 = encoder_roda2;
    msg_stm32_to_pc.encoder_roda3 = encoder_roda3;
    msg_stm32_to_pc.gyroscope = gyroscope;
    msg_stm32_to_pc.battery_voltage = battery_voltage;
    msg_stm32_to_pc.battery_soc = battery_soc;
    msg_stm32_to_pc.tombol = 0x01 + 0x02;
    // ----
    pub_stm32_to_pc->publish(msg_stm32_to_pc);

    return true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_io_stm32 = std::make_shared<IOSTM32>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_io_stm32);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}