#define ZMQ_BUILD_DRAFT_API
#include <memory>
#include <zmq.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "bobik_bridge.hpp"

using std::placeholders::_1;

void *radio;
zmq_msg_t zmq_msg;
char m[] = "honza.";

BobikBridge::BobikBridge() : Node("bobik_bridge")
{
  sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&BobikBridge::cmd_vel_callback, this, _1));
}

void BobikBridge::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", "msg->data.c_str()");


  char topic[] = "cmd_vel";
  m[5] = 0;
  if (zmq_msg_init_data(&zmq_msg, m, strlen(m), NULL, NULL) == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to init message for topic %s. %s", topic, zmq_strerror(errno));
    return;
  }
  if (zmq_msg_set_group(&zmq_msg, topic) == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set group for topic %s. %s", topic, zmq_strerror(errno));
    return;
  }
  if (zmq_msg_send(&zmq_msg, radio, 0) == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send message for topic %s. %s", topic, zmq_strerror(errno));
    return;
  }
  if (zmq_msg_close(&zmq_msg) == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send message for topic %s. %s", topic, zmq_strerror(errno));
    return;
  }
}

rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;

int main(int argc, char *argv[])
{
  // Setup 0MQ
  void *zmq_ctx = zmq_ctx_new();
  radio = zmq_socket(zmq_ctx, ZMQ_RADIO);

  int nula = 0;
  if (zmq_setsockopt(radio, ZMQ_LINGER, &nula, sizeof(nula)) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("bobik_bridge"), "zmq_setsockopt ZMQ_LINGER: %s", zmq_strerror(errno));
    return EXIT_FAILURE;
  }
  int jedna = 1; // important to set queue length to 1. It will help to drop superseeded messages when not sent yet.
  if (zmq_setsockopt(radio, ZMQ_SNDHWM, &jedna, sizeof(jedna)) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("bobik_bridge"), "zmq_setsockopt ZMQ_SNDHWM: %s", zmq_strerror(errno));
    return EXIT_FAILURE;
  }

  if (zmq_connect(radio, "udp://127.0.0.1:7654") != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("bobik_bridge"), "zmq_connect: %s", zmq_strerror(errno));
    return EXIT_FAILURE;
  }

  // Setup ROS2
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BobikBridge>());

  // Cleanup
  rclcpp::shutdown();
  zmq_close(radio);
  zmq_term(zmq_ctx);

  return EXIT_SUCCESS;
}
