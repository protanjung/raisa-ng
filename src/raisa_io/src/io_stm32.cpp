#include "pandu_ros2_kit/help_logger.hpp"
#include "pandu_ros2_kit/help_udp.hpp"
#include "raisa_interfaces/msg/stm32_from_pc.hpp"
#include "raisa_interfaces/msg/stm32_to_pc.hpp"
#include "rclcpp/rclcpp.hpp"

class IOSTM32 : public rclcpp::Node {
  public:
    //----Parameter
    std::string ip;
    int port;
    //----Timer
    rclcpp::TimerBase::SharedPtr tim_50hz;
    //----Subscriber
    rclcpp::Subscription<raisa_interfaces::msg::Stm32FromPc>::SharedPtr sub_stm32_from_pc;
    //----Publisher
    rclcpp::Publisher<raisa_interfaces::msg::Stm32ToPc>::SharedPtr pub_stm32_to_pc;
    //----Help
    HelpLogger logger;
    HelpUDP udp;

    // Socket connection
    // -----------------
    uint8_t tx_buffer[1024];
    uint8_t rx_buffer[1024];

    // Data from PC
    // ------------
    uint32_t epoch_from_pc;
    uint8_t mode;
    int16_t dx;
    int16_t dy;
    int16_t dtheta;

    // Data to PC
    // ----------
    uint32_t epoch_to_pc;
    uint16_t encoder_roda0;
    uint16_t encoder_roda1;
    uint16_t encoder_roda2;
    uint16_t encoder_roda3;
    float gyroscope;
    float battery_voltage;
    float battery_soc;
    uint8_t battery_charging;
    uint16_t fault_code;

    IOSTM32() : Node("io_stm32") {
        //----Parameter
        this->declare_parameter("ip", rclcpp::PARAMETER_STRING);
        this->declare_parameter("port", rclcpp::PARAMETER_INTEGER);
        this->get_parameter("ip", ip);
        this->get_parameter("port", port);
        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&IOSTM32::cllbck_tim_50hz, this));
        //----Subscriber
        sub_stm32_from_pc = this->create_subscription<raisa_interfaces::msg::Stm32FromPc>(
            "stm32/from_pc", 10, std::bind(&IOSTM32::cllbck_sub_stm32_from_pc, this, std::placeholders::_1));
        //----Publisher
        pub_stm32_to_pc = this->create_publisher<raisa_interfaces::msg::Stm32ToPc>("stm32/to_pc", 10);
        //----Help
        if (!logger.init()) {
            rclcpp::shutdown();
        }
        if (!udp.init_as_client(ip, port, true)) {
            rclcpp::shutdown();
        }
    }

    void
    cllbck_tim_50hz() {
        // Communication protocol (PC -> STM32)
        // ------------------------------------
        // Offset | Size | Description
        // -------|------|------------
        // 0      | 4    | epoch_from_pc
        // 4      | 1    | mode
        // 5      | 2    | dx
        // 7      | 2    | dy
        // 9      | 2    | dtheta
        memcpy(tx_buffer + 0, &epoch_from_pc, 4);
        memcpy(tx_buffer + 4, &mode, 1);
        memcpy(tx_buffer + 5, &dx, 2);
        memcpy(tx_buffer + 7, &dy, 2);
        memcpy(tx_buffer + 9, &dtheta, 2);

        int len_data_from_pc = udp.send(tx_buffer, 11);
        int len_data_to_pc = udp.recv(rx_buffer, 27);

        // Communication protocol (STM32 -> PC)
        // ------------------------------------
        // Offset | Size | Description
        // -------|------|------------
        // 0      | 4    | epoch_to_pc
        // 4      | 2    | encoder_roda0
        // 6      | 2    | encoder_roda1
        // 8      | 2    | encoder_roda2
        // 10     | 2    | encoder_roda3
        // 12     | 4    | gyroscope
        // 16     | 4    | battery_voltage
        // 20     | 4    | battery_soc
        // 24     | 1    | battery_charging
        // 25     | 2    | fault_code
        memcpy(&epoch_to_pc, rx_buffer + 0, 4);
        memcpy(&encoder_roda0, rx_buffer + 4, 2);
        memcpy(&encoder_roda1, rx_buffer + 6, 2);
        memcpy(&encoder_roda2, rx_buffer + 8, 2);
        memcpy(&encoder_roda3, rx_buffer + 10, 2);
        memcpy(&gyroscope, rx_buffer + 12, 4);
        memcpy(&battery_voltage, rx_buffer + 16, 4);
        memcpy(&battery_soc, rx_buffer + 20, 4);
        memcpy(&battery_charging, rx_buffer + 24, 1);
        memcpy(&fault_code, rx_buffer + 25, 2);

        if (len_data_from_pc < 0 || len_data_to_pc < 0) {
            return;
        }

        // -----------------------------

        epoch_from_pc++;

        static uint32_t epoch_to_pc_old = epoch_to_pc - 1;
        if (epoch_to_pc_old != epoch_to_pc - 1) {
            epoch_to_pc_old = epoch_to_pc;
            return;
        }

        epoch_to_pc_old = epoch_to_pc;

        // -----------------------------

        static raisa_interfaces::msg::Stm32ToPc msg_stm32_to_pc;
        msg_stm32_to_pc.encoder_roda0 = encoder_roda0;
        msg_stm32_to_pc.encoder_roda1 = encoder_roda1;
        msg_stm32_to_pc.encoder_roda2 = encoder_roda2;
        msg_stm32_to_pc.encoder_roda3 = encoder_roda3;
        msg_stm32_to_pc.gyroscope = gyroscope;
        msg_stm32_to_pc.battery_voltage = battery_voltage;
        msg_stm32_to_pc.battery_soc = battery_soc;
        msg_stm32_to_pc.battery_charging = battery_charging;
        msg_stm32_to_pc.fault_code = fault_code;
        pub_stm32_to_pc->publish(msg_stm32_to_pc);
    }

    void
    cllbck_sub_stm32_from_pc(const raisa_interfaces::msg::Stm32FromPc::SharedPtr msg) {
        mode = msg->mode;
        dx = msg->dx;
        dy = msg->dy;
        dtheta = msg->dtheta;
    }
};

int
main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node_io_stm32 = std::make_shared<IOSTM32>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_io_stm32);
    executor.spin();

    return 0;
}