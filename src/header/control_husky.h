#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#define MANUAL
class Navigation : public rclcpp :: Node
{
    public:
        Navigation();

    private:
        #ifdef MANUAL
            void ReadSensor(const sensor_msgs::msg::Joy::SharedPtr msg);
            void check_joystick_connection();
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystic_sub; //create_sub shared pointer returns
            bool is_joy_connected;
            rclcpp::Time last_joystick_data;
            rclcpp::TimerBase::SharedPtr watchdog_timer; 
       #endif
       rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

};