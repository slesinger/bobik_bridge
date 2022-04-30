#ifndef BOBIK_DRIVER_HPP_
#define BOBIK_DRIVER_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

class BobikBridge : public rclcpp::Node
{
public:
    BobikBridge();

private:
    std::shared_future<void> future_;
    std::promise<void> exit_signal_;
    std::thread zmq_read_thread_;
    void zmq_read_thread_func(const std::shared_future<void> &local_future);
    void send_to_zmq_topic(const char *topic, uint8_t *data, size_t size) const;
    void cmd_vel_torobot(const geometry_msgs::msg::Twist::SharedPtr msg) const;
    void caster_raw_fromrobot(const void *data_buffer) const;
    /**
     * @brief Calculate odometry from drive joint states.
     *
     * @param MsgCasterJointStates_t data
     * @return nav_msgs::msg::Odometry message
     */
    nav_msgs::msg::Odometry calculate_odom(std::vector<int16_t> *data) const;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_raw_caster;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
};

#endif