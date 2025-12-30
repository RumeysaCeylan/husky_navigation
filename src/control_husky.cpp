#include "header/control_husky.h"

Navigation::Navigation() : Node("navigation")
{
    #ifdef MANUAL
    RCLCPP_INFO(this->get_logger(),"Manual Driving");
    joystic_sub = create_subscription<sensor_msgs::msg::Joy>("/joy",
        10, std::bind(&Navigation::ReadSensor, this, std::placeholders::_1));
        is_joy_connected = false;
    twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/a200_0000/cmd_vel",10);
        watchdog_timer = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&Navigation::check_joystick_connection, this)
        );
        
    #endif
}
#ifdef MANUAL
void Navigation::ReadSensor(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    geometry_msgs::msg::Twist twist;
    last_joystick_data = this->now();

    if(!is_joy_connected)
    {
        RCLCPP_INFO(this->get_logger(),"Joystick connected");
        is_joy_connected = true;
    }
    twist.linear.x = msg->axes[1];

    twist.linear.y = msg->axes[3];

    twist.angular.z = -1.0*msg->axes[0];
    RCLCPP_INFO(this->get_logger(), "Joy msg angular.z: %f\r linear.x: %f \r linear.y : %f\r\n", msg->axes[0], msg->axes[1], msg->axes[3]);
    twist_pub->publish(twist);
}

void Navigation::check_joystick_connection()
{
    auto now = this->now();

    auto duration = now - last_joystick_data;
    if(duration.seconds() > 5.0){
        if(is_joy_connected)
        {
            RCLCPP_INFO(this->get_logger(),"Joystick disconnected");
            is_joy_connected = false;
        }
    }
}
#endif

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Navigation>());
    rclcpp::shutdown();
    return 0;
}