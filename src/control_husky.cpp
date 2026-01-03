#include "header/control_husky.h"

Navigation::Navigation() : Node("navigation")
{
#ifdef MANUAL
    RCLCPP_INFO(this->get_logger(), "Manual Driving");
    joystic_sub = create_subscription<sensor_msgs::msg::Joy>("/joy",
                                                             10, std::bind(&Navigation::ReadSensor, this, std::placeholders::_1));

    is_joy_connected = false;

    watchdog_timer = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&Navigation::check_joystick_connection, this));

#endif

#ifdef AUTONOMOUS
    RCLCPP_INFO(this->get_logger(), "AUTONOMUS Driving");
    laser_sub = create_subscription<sensor_msgs::msg::LaserScan>("/a200_0000/sensors/lidar2d_0/scan",
                                                                 10, std::bind(&Navigation::ReadLidar, this, std::placeholders::_1));
#endif
    twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/a200_0000/cmd_vel", 10);
}
#ifdef MANUAL
void Navigation::ReadSensor(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    geometry_msgs::msg::Twist twist;
    last_joystick_time_data = steady_clock_.now();

    if (!is_joy_connected)
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32mJoystick connected\033[0m");
        is_joy_connected = true;
    }
    twist.linear.x = msg->axes[1];

    twist.linear.y = msg->axes[3];

    twist.angular.z = -1.0 * msg->axes[0];
    RCLCPP_INFO(this->get_logger(), "Joy msg angular.z: %f linear.x: %f linear.y : %f\r\n", msg->axes[0], msg->axes[1], msg->axes[3]);
    twist_pub->publish(twist);
}

void Navigation::check_joystick_connection()
{
    auto now = steady_clock_.now();
    RCLCPP_ERROR(get_logger(), "clock now=%d last=%d",
                 (int)now.get_clock_type(), (int)last_joystick_time_data.get_clock_type());

    auto duration = now - last_joystick_time_data;
    if (duration.seconds() > 5.0)
    {
        if (is_joy_connected)
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;33mJoystick disconnected\033[0m");
            is_joy_connected = false;
        }
    }
}
#endif
#ifdef AUTONOMOUS
void Navigation::ReadLidar(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Lidar angle_min: %f angle_max : %f angle_inc : %f\r\n", msg->angle_min, msg->angle_max, msg->angle_increment);
    LidarData lidarData;

    geometry_msgs::msg::Twist twist;

    for (int i = 0; i < 6; i++)
        lidarData.min_distance[i] = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        float angle = msg->angle_min + i * msg->angle_increment;

        float dist = msg->ranges[i];
        if (!std::isfinite(dist))
            continue;
     

        if (angle >= -0.26 && angle <= 0.26)
        {

            lidarData.dir = FRONT;
            lidarData.min_distance[lidarData.dir] = std::min(lidarData.min_distance[lidarData.dir], dist);
        }
        if (angle >= -0.78 && angle <= -0.26)
        {

            // RCLCPP_ERROR(get_logger(), "FRONT RIGHT OBSTACLE");
            lidarData.dir = FRONT_RIGHT;
            lidarData.min_distance[lidarData.dir] = std::min(lidarData.min_distance[lidarData.dir], dist);
        }
        if (angle >= -1.57 && angle <= -0.78)
        {

            // RCLCPP_ERROR(get_logger(), "RIGHT OBSTACLE");
            lidarData.dir = RIGHT;
            lidarData.min_distance[lidarData.dir] = std::min(lidarData.min_distance[lidarData.dir], dist);
        }
        if (angle >= 0.26 && angle <= 0.78)
        {

            // RCLCPP_ERROR(get_logger(), "FRONT LEFT OBSTACLE");
            lidarData.dir = FRONT_LEFT;
            lidarData.min_distance[lidarData.dir] = std::min(lidarData.min_distance[lidarData.dir], dist);
        }
        if (angle >= 0.78 && angle <= 1.57)
        {

            // RCLCPP_ERROR(get_logger(), "LEFT OBSTACLE");
            lidarData.dir = LEFT;
            lidarData.min_distance[lidarData.dir] = std::min(lidarData.min_distance[lidarData.dir], dist);
        }
    }

    const float front_dist = lidarData.min_distance[FRONT];
    const float left_gap = std::max(lidarData.min_distance[FRONT_LEFT], lidarData.min_distance[LEFT]);
    const float right_gap = std::max(lidarData.min_distance[FRONT_RIGHT], lidarData.min_distance[RIGHT]);

    // for (DIRECTION dir : {FRONT, FRONT_RIGHT, FRONT_LEFT, RIGHT, LEFT, BACK})
    // {
    //     if (lidarData.min_distance[dir] < 1.0)
    //     {
    //         RCLCPP_ERROR(get_logger(), "%s OBSTACLE", direction_str(dir));
    //         twist.linear.x = 0.4;
    //         // twist.angular.z =
    //     }
    //     else if (lidarData.min_distance[dir] < 0.5 && dir != FRONT)
    //     {
    //         RCLCPP_ERROR(get_logger(), "%s OBSTACLE", direction_str(dir));
    //     }
    // }
    if (front_dist > 2.0)
    {
        twist.linear.x = 0.6; 
        twist.angular.z = 0.0;
    }
    else
    {
        const bool turn_left = (left_gap >= right_gap);

        const float w = (front_dist < 1.0) ? 0.9f : 0.5f;
        if(turn_left)
        {
            twist.linear.x = (front_dist < 0.5) ? 0.0 : 0.2;
            twist.angular.z = w;

        }
        else
        {
            twist.linear.x = (front_dist < 0.5) ? 0.0 : 0.2;
            twist.angular.z = w;
        }
        

    }
    twist_pub->publish(twist);

    RCLCPP_INFO(this->get_logger(),
                "front=%.2f left_gap=%.2f right_gap=%.2f -> v=%.2f w=%.2f",
                front_dist, left_gap, right_gap, twist.linear.x, twist.angular.z);
}
const char *Navigation::direction_str(DIRECTION dir)
{
    switch (dir)
    {
    case FRONT:
        return "FRONT";
    case FRONT_RIGHT:
        return "FRONT_RIGHT";
    case FRONT_LEFT:
        return "FRONT_LEFT";
    case RIGHT:
        return "RIGHT";
    case LEFT:
        return "LEFT";
    case BACK:
        return "BACK";
    default:
        return "UNKNOWN";
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