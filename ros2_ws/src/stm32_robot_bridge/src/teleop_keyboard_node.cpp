#include <rclcpp/rclcpp.hpp>
#include "stm32_robot_bridge/msg/motor_command.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <cctype>

class TeleopKeyboardNode : public rclcpp::Node
{
public:
    TeleopKeyboardNode() : Node("teleop_keyboard_node")
    {
        publisher_ = this->create_publisher<stm32_robot_bridge::msg::MotorCommand>("motor_commands", 10);
        RCLCPP_INFO(this->get_logger(), "Keyboard Teleop Started.\n"
                                        "Controls:\n"
                                        "W - Forward\n"
                                        "S - Backward\n"
                                        "A - Left (Pivot)\n"
                                        "D - Right (Pivot)\n"
                                        "1-9 - Speed 10%%-100%% (Default 50%%)\n"
                                        "SPACE - Emergency STOP\n"
                                        "Press CTRL+C to quit.");

        // Start non-blocking input thread
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TeleopKeyboardNode::timer_callback, this));
            
        // Setup terminal
        tcgetattr(STDIN_FILENO, &oldt_);
        newt_ = oldt_;
        newt_.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);
        
        // Non-blocking read
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    ~TeleopKeyboardNode() {
        // Restore terminal
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    }

private:
    rclcpp::Publisher<stm32_robot_bridge::msg::MotorCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios oldt_, newt_;
    float current_speed_ = 50.0f;

    void publish_command(uint8_t motor_id, float speed, uint8_t direction) {
        auto msg = stm32_robot_bridge::msg::MotorCommand();
        msg.motor_id = motor_id;
        msg.speed = speed;
        msg.direction = direction;
        publisher_->publish(msg);
    }

    void handle_movement(char c) {
        switch (std::tolower(static_cast<unsigned char>(c))) {
            case 'w': // Forward
                publish_command(255, current_speed_, 1); // 255 = ALL
                RCLCPP_INFO(this->get_logger(), "Forward");
                break;
            case 's': // Backward
                publish_command(255, current_speed_, 0);
                RCLCPP_INFO(this->get_logger(), "Backward");
                break;
            case 'a': // Pivot Left (Left motors backward, Right motors forward)
                publish_command(1, current_speed_, 0); // Assuming 1,2 are left
                publish_command(2, current_speed_, 0);
                publish_command(3, current_speed_, 1); // Assuming 3,4 are right
                publish_command(4, current_speed_, 1);
                RCLCPP_INFO(this->get_logger(), "Pivot Left");
                break;
            case 'd': // Pivot Right
                publish_command(1, current_speed_, 1);
                publish_command(2, current_speed_, 1);
                publish_command(3, current_speed_, 0);
                publish_command(4, current_speed_, 0);
                RCLCPP_INFO(this->get_logger(), "Pivot Right");
                break;
            case ' ': // Stop
                publish_command(255, 0.0, 2);
                RCLCPP_INFO(this->get_logger(), "STOP ALL");
                break;
            default:
                if (c >= '1' && c <= '9') {
                    current_speed_ = (c - '0') * 10.0f; // 1->10%, 9->90%
                    RCLCPP_INFO(this->get_logger(), "Speed set to %.0f%%", current_speed_);
                }
                break;
        }
    }

    void timer_callback() {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            handle_movement(c);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopKeyboardNode>());
    rclcpp::shutdown();
    return 0;
}
