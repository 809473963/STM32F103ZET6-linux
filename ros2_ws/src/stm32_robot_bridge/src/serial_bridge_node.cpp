#include <rclcpp/rclcpp.hpp>
#include "stm32_robot_bridge/msg/motor_command.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

using std::placeholders::_1;

class SerialBridgeNode : public rclcpp::Node
{
public:
    SerialBridgeNode() : Node("serial_bridge_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string port_name;
        int baud_rate;
        this->get_parameter("port_name", port_name);
        this->get_parameter("baud_rate", baud_rate);

        init_serial(port_name, baud_rate);

        subscription_ = this->create_subscription<stm32_robot_bridge::msg::MotorCommand>(
            "motor_commands", 10, std::bind(&SerialBridgeNode::topic_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Serial Bridge Node started on %s at %d", port_name.c_str(), baud_rate);
    }

    ~SerialBridgeNode() {
        if (serial_fd_ >= 0) close(serial_fd_);
    }

private:
    int serial_fd_ = -1;
    rclcpp::Subscription<stm32_robot_bridge::msg::MotorCommand>::SharedPtr subscription_;

    void init_serial(const std::string& port, int baud) {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            return;
        }

        struct termios options;
        tcgetattr(serial_fd_, &options);

        speed_t speed = B115200;
        switch (baud) {
            case 9600: speed = B9600; break;
            case 115200: speed = B115200; break;
            default: RCLCPP_WARN(this->get_logger(), "Unsupported baud rate, defaulting to 115200"); break;
        }

        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, set local mode
        options.c_cflag &= ~PARENB;          // No parity
        options.c_cflag &= ~CSTOPB;          // 1 stop bit
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;              // 8 data bits

        options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable flow control
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
        options.c_oflag &= ~OPOST;           // Raw output

        tcsetattr(serial_fd_, TCSANOW, &options);
        fcntl(serial_fd_, F_SETFL, FNDELAY);
    }

    void topic_callback(const stm32_robot_bridge::msg::MotorCommand::SharedPtr msg)
    {
        if (serial_fd_ < 0) return;

        // Construct STM32 protocol frame: [HEAD] [LEN] [CMD] [ID] [SPEED] [DIR] [CHECKSUM]
        uint8_t frame[7];
        frame[0] = 0xAA; // Header
        frame[1] = 0x03; // Payload length (id, speed, dir)
        
        bool is_stop_all = (msg->motor_id == 255 && msg->direction == 2);
        frame[2] = is_stop_all ? 0x02 : 0x01; // CMD_STOP_ALL vs CMD_SET_MOTOR_SPEED
        
        frame[3] = msg->motor_id;
        frame[4] = (uint8_t)msg->speed;
        frame[5] = msg->direction;

        // Checksum calculation (Sum of LEN + CMD + DATA)
        uint8_t checksum = frame[1] + frame[2] + frame[3] + frame[4] + frame[5];
        frame[6] = checksum;

        write(serial_fd_, frame, 7);
        // RCLCPP_INFO(this->get_logger(), "Sent %d bytes to serial", 7);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
