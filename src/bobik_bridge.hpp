#ifndef BOBIK_DRIVER_HPP_
#define BOBIK_DRIVER_HPP_

class BobikBridge : public rclcpp::Node
{
  public:
    BobikBridge();

  private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
};

#endif