#include "pandu_ros2_kit/udp.hpp"
#include "raisa_interfaces/msg/basestation_from_pc.hpp"
#include "raisa_interfaces/msg/basestation_to_pc.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class IOBasestation : public rclcpp::Node {
 public:
  //-----Parameter
  int basestation_port;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_1000hz;
  //-----Subscriber
  rclcpp::Subscription<raisa_interfaces::msg::BasestationFromPc>::SharedPtr sub_basestation_from_pc;
  //-----Publisher
  rclcpp::Publisher<raisa_interfaces::msg::BasestationToPc>::SharedPtr pub_basestation_to_pc;

  // Socket connection
  // =================
  UDP basestation_udp;
  uint8_t basestation_tx_buffer[2048];
  uint8_t basestation_rx_buffer[2048];

  // Data to PC
  // ==========
  int16_t dx;
  int16_t dy;
  int16_t dtheta;
  uint16_t tombol;

  // Data from PC
  // ============
  uint8_t status;
  float x;
  float y;
  float theta;
  float battery_voltage;
  float battery_current;
  float battery_soc;
  uint8_t battery_charging;

  IOBasestation() : Node("io_basestation") {
    //-----Parameter
    this->declare_parameter("basestation.port", rclcpp::PARAMETER_INTEGER);
    this->get_parameter("basestation.port", basestation_port);
    //-----Timer
    tim_1000hz = this->create_wall_timer(1ms, std::bind(&IOBasestation::cllbck_tim_1000hz, this));
    //-----Subscriber
    sub_basestation_from_pc = this->create_subscription<raisa_interfaces::msg::BasestationFromPc>(
        "basestation/from_pc",
        10,
        std::bind(&IOBasestation::cllbck_sub_basestation_from_pc, this, std::placeholders::_1));
    //-----Publisher
    pub_basestation_to_pc = this->create_publisher<raisa_interfaces::msg::BasestationToPc>("basestation/to_pc", 10);

    if (basestation_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "Basestation init failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_1000hz() {
    if (basestation_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "Basestation routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_sub_basestation_from_pc(const raisa_interfaces::msg::BasestationFromPc::SharedPtr msg) {
    status = msg->status;
    x = msg->x;
    y = msg->y;
    theta = msg->theta;
    battery_voltage = msg->battery_voltage;
    battery_current = msg->battery_current;
    battery_soc = msg->battery_soc;
    battery_charging = msg->battery_charging;

    std::string delimiter = "\n";
    std::string token = "";
    
    size_t pos = 0;
    while ((pos = msg->log.find(delimiter)) != std::string::npos) {
      token = msg->log.substr(0, pos);

      uint8_t data[1024] = {0};
      data[0] = data[1] = data[2] = data[3] = 0xff;
      memcpy(data + 4, token.c_str(), token.length());

      basestation_udp.send(data, token.length() + 4);

      msg->log.erase(0, pos + delimiter.length());
    }
  }

  //====================================

  bool basestation_init() {
    RCLCPP_INFO(this->get_logger(), "Port: %d", basestation_port);

    if (basestation_udp.init_as_server(basestation_port) == false) {
      RCLCPP_ERROR(this->get_logger(), "Basestation UDP init failed");
      return false;
    }

    return true;
  }

  bool basestation_routine() {
    int len_data_to_pc = -1;
    int len_data_from_pc = -1;

    if ((len_data_to_pc = basestation_udp.recv(basestation_rx_buffer, sizeof(basestation_rx_buffer))) >= 0) {
      // Communication Protocol (Basestation -> PC)
      // ==========================================
      // Offset   | Size  | Description
      // 0        | 2     | dx
      // 2        | 2     | dy
      // 4        | 2     | dtheta
      // 6        | 2     | tombol
      memcpy(&dx, basestation_rx_buffer + 0, 2);
      memcpy(&dy, basestation_rx_buffer + 2, 2);
      memcpy(&dtheta, basestation_rx_buffer + 4, 2);
      memcpy(&tombol, basestation_rx_buffer + 6, 2);

      // Communication Protocol (PC -> Basestation)
      // ==========================================
      // Offset   | Size  | Description
      // 0        | 1     | status
      // 1        | 4     | x
      // 5        | 4     | y
      // 9        | 4     | theta
      // 13       | 4     | battery_voltage
      // 17       | 4     | battery_current
      // 21       | 4     | battery_soc
      // 25       | 1     | battery_charging
      memcpy(basestation_tx_buffer + 0, &status, 1);
      memcpy(basestation_tx_buffer + 1, &x, 4);
      memcpy(basestation_tx_buffer + 5, &y, 4);
      memcpy(basestation_tx_buffer + 9, &theta, 4);
      memcpy(basestation_tx_buffer + 13, &battery_voltage, 4);
      memcpy(basestation_tx_buffer + 17, &battery_current, 4);
      memcpy(basestation_tx_buffer + 21, &battery_soc, 4);
      memcpy(basestation_tx_buffer + 25, &battery_charging, 1);

      len_data_from_pc = basestation_udp.send(basestation_tx_buffer, 26);
    }

    // =================================

    static rclcpp::Time time_old = this->now();

    if (len_data_from_pc >= 0) { time_old = this->now(); }

    if (this->now() - time_old > 1s) {
      dx = 0;
      dy = 0;
      dtheta = 0;
      tombol = 0;
    }

    // =================================

    static raisa_interfaces::msg::BasestationToPc msg_basestation_to_pc;
    // ----
    msg_basestation_to_pc.dx = dx;
    msg_basestation_to_pc.dy = dy;
    msg_basestation_to_pc.dtheta = dtheta;
    msg_basestation_to_pc.tombol = tombol;
    // ----
    pub_basestation_to_pc->publish(msg_basestation_to_pc);

    return true;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node_io_basestation = std::make_shared<IOBasestation>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_io_basestation);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}