#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
//#define MANUAL
#define AUTONOMOUS
enum DIRECTION
{
    FRONT = 0,
    FRONT_RIGHT,
    FRONT_LEFT,
    RIGHT,
    LEFT,
    BACK
};
struct LidarData
{
    float min_distance[6];
    DIRECTION dir;
};
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
            rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

            rclcpp::Time last_joystick_time_data{0, 0, RCL_STEADY_TIME};
            rclcpp::TimerBase::SharedPtr watchdog_timer; 
       #endif
       #ifdef AUTONOMOUS
            void ReadLidar(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub; 
            const char* direction_str(DIRECTION dir);
       #endif
       rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

};