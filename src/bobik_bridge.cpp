#define ZMQ_BUILD_DRAFT_API
#include <future>
#include <memory>
#include <zmq.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "bobik_bridge.hpp"
#include "protocol_types.h"

#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

void *radio;
void *dish;
zmq_msg_t zmq_msg;

BobikBridge::BobikBridge() : rclcpp::Node("bobik_bridge")
{
    future_ = exit_signal_.get_future();
    sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        TOPIC_CMD_VEL, 10, std::bind(&BobikBridge::cmd_vel_torobot, this, _1));
    pub_raw_caster = this->create_publisher<std_msgs::msg::Int16MultiArray>("driver/raw/caster", 10);
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    zmq_read_thread_ = std::thread(&BobikBridge::zmq_read_thread_func, this, future_);
}

void BobikBridge::send_to_zmq_topic(const char *topic, uint8_t *data, size_t size) const
{
    if (zmq_msg_init_data(&zmq_msg, data, size, NULL, NULL) == -1)
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

void BobikBridge::zmq_read_thread_func(const std::shared_future<void> &local_future)
{
    std::future_status status;

    do
    {
        int bytesReceived;
        zmq_msg_t receiveMessage;

        zmq_msg_init(&receiveMessage);
        bytesReceived = zmq_msg_recv(&receiveMessage, dish, 0);
        if (bytesReceived == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bobik_bridge"), "Failed to receive message.");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("bobik_bridge"), "topic: %s, data: %s, size: %d\n", zmq_msg_group(&receiveMessage), (char *)zmq_msg_data(&receiveMessage), bytesReceived);
            if (strcmp(zmq_msg_group(&receiveMessage), TOPIC_CASTER_RAW) == 0)
            {
                void *data_buffer = zmq_msg_data(&receiveMessage);
                MsgCasterJointStates_t *msg = (struct MsgCasterJointStates_t *)data_buffer;
                std::vector<int16_t> data = {
                    msg->fl_caster_rotation_joint,
                    msg->fl_caster_drive_joint,
                    msg->fr_caster_rotation_joint,
                    msg->fr_caster_drive_joint,
                    msg->r_caster_rotation_joint,
                    msg->r_caster_drive_joint};
                auto message = std_msgs::msg::Int16MultiArray();
                message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
                message.layout.dim[0].label = "rot";
                message.layout.dim[0].size = data.size();
                message.layout.dim[0].stride = 1;
                message.layout.data_offset = 0;
                message.data = data;
                pub_raw_caster->publish(message);
                pub_odom->publish(calculate_odom(&data));
            }
        }

        zmq_msg_close(&receiveMessage);

        status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
}

void BobikBridge::cmd_vel_torobot(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "From ROS2 /cmd_vel: '%s'", "msg->data.c_str()");

    geometry_msgs::msg::Vector3 linear = msg->linear;
    geometry_msgs::msg::Vector3 angular = msg->angular;
    MsgCmdVel_t msg_cmd_vel;
    msg_cmd_vel.linear_x = (int16_t)(linear.x * FLOAT_INT16_PRECISION);
    msg_cmd_vel.linear_y = (int16_t)(linear.y * FLOAT_INT16_PRECISION);
    msg_cmd_vel.rotation = (int16_t)(angular.z * FLOAT_INT16_PRECISION);
    send_to_zmq_topic(TOPIC_CMD_VEL, (uint8_t *)(&msg_cmd_vel), 6);
}

void BobikBridge::caster_raw_fromrobot(const void *data_buffer) const
{
    RCLCPP_INFO(this->get_logger(), "From ZMQ /caster_raw_joint_states: '%s'", "msg->data.c_str()");
    MsgCasterJointStates_t *msg = (struct MsgCasterJointStates_t *)data_buffer;
    std::vector<int16_t> data = {
        msg->fl_caster_rotation_joint,
        msg->fl_caster_drive_joint,
        msg->fr_caster_rotation_joint,
        msg->fr_caster_drive_joint,
        msg->r_caster_rotation_joint,
        msg->r_caster_drive_joint};
    auto message = std_msgs::msg::Int16MultiArray();
    message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    message.layout.dim[0].label = "rot";
    message.layout.dim[0].size = data.size();
    message.layout.dim[0].stride = 1;
    message.layout.data_offset = 0;
    message.data = data;
    pub_raw_caster->publish(message);
    pub_odom->publish(calculate_odom(&data));
    // RCLCPP_INFO(this->get_logger(), "Caster rotation %d : %d : %d", msg->fl_caster_rotation_joint, msg->fr_caster_rotation_joint, msg->r_caster_rotation_joint);
}

nav_msgs::msg::Odometry BobikBridge::calculate_odom(std::vector<int16_t> *data) const
{
#define LEN_AB 0.457 // distance between twa caster axis
#define LEN_AB_HALF LEN_AB / 2.0
#define LEN_Cc LEN_AB * 1.732051 / 2.0 // height of same-side triangle, distance between point C and center of line c (AB)
#define LEN_SC 2.0 / 3.0 * LEN_Cc      // distance between robot center (S - stred) and rear caster axis C
#define LEN_Sc 1.0 / 3.0 * LEN_Cc      // distance between robot center (S - stred) and center of line c (AB)
#define POS_A_x LEN_Sc
#define POS_A_y -LEN_AB_HALF
#define POS_B_x LEN_Sc
#define POS_B_y LEN_AB_HALF
#define POS_C_x -LEN_SC
#define POS_C_y 0.0
#define CASTER_UNITS2RAD M_PI / -8192.0
#define CASTER_TICKS2METERS (0.123 * M_PI) / (2 * 120)

    MsgCasterJointStates_t *caster_data = (struct MsgCasterJointStates_t *)data->data();
    // Front Left caster new position
    float Arad = CASTER_UNITS2RAD * caster_data->fl_caster_rotation_joint;
    float Ameters = CASTER_TICKS2METERS * caster_data->fl_caster_drive_joint;
    float Ax = POS_A_x + cos(Arad) * Ameters;
    float Ay = POS_A_y - sin(Arad) * Ameters;
    // Front Right caster new position
    float Brad = CASTER_UNITS2RAD * caster_data->fr_caster_rotation_joint;
    float Bmeters = CASTER_TICKS2METERS * caster_data->fr_caster_drive_joint;
    float Bx = POS_B_x + cos(Brad) * Bmeters;
    float By = POS_B_y - sin(Brad) * Bmeters;
    // Rear caster new position
    float Crad = CASTER_UNITS2RAD * caster_data->r_caster_rotation_joint;
    float Cmeters = CASTER_TICKS2METERS * caster_data->r_caster_drive_joint;
    float Cx = POS_C_x + cos(Crad) * Cmeters;
    float Cy = POS_C_y - sin(Crad) * Cmeters;
    // Center position between front left and right
    float c_center_x = (Ax + Bx) / 2.0;
    float c_center_y = (Ay + By) / 2.0;
    // Vector between c center and rear
    float c_center_to_rear_x = Cx - c_center_x;
    float c_center_to_rear_y = Cy - c_center_y;
    // New base center position
    float base_center_x = c_center_x + c_center_to_rear_x / 3.0;
    float base_center_y = c_center_y + c_center_to_rear_y / 3.0;
    // New base vector forward
    float base_forward_x = -c_center_to_rear_x;
    float base_forward_y = -c_center_to_rear_y;
    // Rotation of base in radians
    float base_rotation = atan2(base_forward_y, base_forward_x); // assumption: base cannot rotate more than 90 degrees with one frame

    geometry_msgs::msg::Pose odom_pose;
    odom_pose.position.x += base_center_x;
    odom_pose.position.y += base_center_y;
    odom_pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, base_rotation);
    odom_pose.orientation = tf2::toMsg(q);

    nav_msgs::msg::Odometry message;
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    message.header.stamp = rclcpp::Clock().now();
    message.pose.pose = odom_pose;
    message.twist.twist.linear.x = base_center_x;
    message.twist.twist.linear.y = base_center_y;
    message.twist.twist.linear.z = 0;
    message.twist.twist.angular.x = 0;
    message.twist.twist.angular.y = 0;
    message.twist.twist.angular.z = base_rotation;
    return message;
}

int main(int argc, char *argv[])
{
    RCLCPP_INFO(rclcpp::get_logger("bobik_bridge"), "Starting bobik_bridge");

    // Setup 0MQ
    void *zmq_ctx = zmq_ctx_new();
    radio = zmq_socket(zmq_ctx, ZMQ_RADIO);
    dish = zmq_socket(zmq_ctx, ZMQ_DISH);

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
    if (zmq_bind(dish, "udp://*:7655") != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("bobik_bridge"), "zmq_bind: %s", zmq_strerror(errno));
        return EXIT_FAILURE;
    }
    if (zmq_join(dish, TOPIC_CASTER_RAW) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("bobik_bridge"), "Could not subscribe to: %s", TOPIC_CASTER_RAW);
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
