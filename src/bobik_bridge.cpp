#define ZMQ_BUILD_DRAFT_API
#include <future>
#include <memory>
#include <zmq.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "bobik_bridge.hpp"
#include "protocol_types.h"
#include <boost/assign/list_of.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;
using namespace boost::assign;

void *radio;
void *dish;
zmq_msg_t zmq_msg;
float odom_pose_x;
float odom_pose_y;
float odom_pose_gamma;

#define CALIBRATION_FRAMES 100  // how many frames since robot start will be used to calculate bias
#define IMU_STEADY_ACC_TRESHOLD 0.02 //IMU values below this treshold will be reported as 0 to decrease drift
unsigned int calibration_cnt = 0;
double imu_acc_bias_x_accum = 0;
double imu_acc_bias_y_accum = 0;
double imu_acc_bias_x = 0;
double imu_acc_bias_y = 0;

int16_t raw_fl = 0;
int16_t raw_fr = 0;
int16_t raw_r = 0;

#define MSG_TRANSPORT_DELAY_MS 50 // how many milliseconds sooner an event happened before it reached this Bobik Bridge, to compensate
#define DEG2RAD M_PI / 180.0
#define G2MS2 9.80665 / 16384
#define A_TRIANGLE_SIDE 0.53
// These defines are owned by bobik_arduino. This is just a dumb copy.
#define LEN_AB 0.457 // distance between twa caster axis
#define LEN_AB_HALF LEN_AB / 2.0
#define LEN_Cc LEN_AB * 1.7320508075688772935 / 2.0 // height of same-side triangle, distance between point C and center of line c (AB)
#define LEN_SC 2.0 / 3.0 * LEN_Cc                   // distance between robot center (S - stred) and rear caster axis C
#define LEN_Sc 1.0 / 3.0 * LEN_Cc                   // distance between robot center (S - stred) and center of line c (AB)
#define POS_A_x LEN_Sc
#define POS_A_y -LEN_AB_HALF
#define POS_B_x LEN_Sc
#define POS_B_y LEN_AB_HALF
#define POS_C_x -LEN_SC
#define POS_C_y 0.0
#define CASTER_UNITS2RAD M_PI / -8192.0
#define REAL_WORLD_COEF_A 1.028 // coeficient adjusting for real world measurements.
#define REAL_WORLD_COEF_B 1.028
#define REAL_WORLD_COEF_C 1.15
#define CASTER_TICKS_PER_REVOLUTION (2 * 120)
#define CASTER_TICKS2METERS ((0.123 * M_PI) / CASTER_TICKS_PER_REVOLUTION)

BobikBridge::BobikBridge() : rclcpp::Node("bobik_bridge")
{
    future_ = exit_signal_.get_future();
    sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        TOPIC_CMD_VEL, 10, std::bind(&BobikBridge::cmd_vel_torobot, this, _1));
    pub_raw_caster = this->create_publisher<std_msgs::msg::Int16MultiArray>("driver/raw/caster", 10);
    pub_joint_states = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pub_scan = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    // pub_mag = this->create_publisher<sensor_msgs::msg::MagneticField>("magnetic_field", 10);
    fl_caster_wheel_joint = 0;
    fr_caster_wheel_joint = 0;
    r_caster_wheel_joint = 0;

    zmq_read_thread_ = std::thread(&BobikBridge::zmq_read_thread_func, this, future_);
    RCLCPP_INFO(rclcpp::get_logger("bobik_bridge"), "BobikBridge instance created");
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
            // RCLCPP_INFO(rclcpp::get_logger("bobik_bridge"), "topic: %s, size: %d", zmq_msg_group(&receiveMessage), bytesReceived);
            if (strcmp(zmq_msg_group(&receiveMessage), TOPIC_CASTER_RAW) == 0)
            {
                void *data_buffer = zmq_msg_data(&receiveMessage);
                MsgCasterJointStates_t *msg = (struct MsgCasterJointStates_t *)data_buffer;

                // Prepare data for odom calculation
                std::vector<int16_t> data = {
                    msg->fl_caster_rotation_joint,
                    msg->fl_caster_drive_joint,
                    msg->fr_caster_rotation_joint,
                    msg->fr_caster_drive_joint,
                    msg->r_caster_rotation_joint,
                    msg->r_caster_drive_joint};

                raw_fl += msg->fl_caster_drive_joint;
                raw_fr += msg->fr_caster_drive_joint;
                raw_r += msg->r_caster_drive_joint;

                std::vector<int16_t> raw_data = {
                    msg->fl_caster_rotation_joint,
                    raw_fl,
                    msg->fr_caster_rotation_joint,
                    raw_fr,
                    msg->r_caster_rotation_joint,
                    raw_r};

                auto raw_message = std_msgs::msg::Int16MultiArray();
                raw_message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
                raw_message.layout.dim[0].label = "rotdrv";
                raw_message.layout.dim[0].size = raw_data.size();
                raw_message.layout.dim[0].stride = 1;
                raw_message.layout.data_offset = 0;
                raw_message.data = raw_data;

                auto msg_base_joint_states = sensor_msgs::msg::JointState();
                msg_base_joint_states.header.stamp = rclcpp::Clock().now();
                msg_base_joint_states.name.push_back("fl_caster_rotation_joint");
                msg_base_joint_states.name.push_back("fl_caster_wheel_joint");
                msg_base_joint_states.name.push_back("fr_caster_rotation_joint");
                msg_base_joint_states.name.push_back("fr_caster_wheel_joint");
                msg_base_joint_states.name.push_back("r_caster_rotation_joint");
                msg_base_joint_states.name.push_back("r_caster_wheel_joint");
                fl_caster_wheel_joint += msg->fl_caster_drive_joint;
                fr_caster_wheel_joint += msg->fr_caster_drive_joint;
                r_caster_wheel_joint += msg->r_caster_drive_joint;
                // fl_caster_wheel_joint %= CASTER_TICKS_PER_REVOLUTION;
                // fr_caster_wheel_joint %= CASTER_TICKS_PER_REVOLUTION;
                // r_caster_wheel_joint %= CASTER_TICKS_PER_REVOLUTION;
                msg_base_joint_states.position.push_back(msg->fl_caster_rotation_joint * CASTER_UNITS2RAD);
                msg_base_joint_states.position.push_back(2.0 * M_PI * fl_caster_wheel_joint / CASTER_TICKS_PER_REVOLUTION);
                msg_base_joint_states.position.push_back(msg->fr_caster_rotation_joint * CASTER_UNITS2RAD);
                msg_base_joint_states.position.push_back(2.0 * M_PI * fr_caster_wheel_joint / CASTER_TICKS_PER_REVOLUTION);
                msg_base_joint_states.position.push_back(msg->r_caster_rotation_joint * CASTER_UNITS2RAD);
                msg_base_joint_states.position.push_back(2.0 * M_PI * r_caster_wheel_joint / CASTER_TICKS_PER_REVOLUTION);

                pub_raw_caster->publish(raw_message);
                pub_joint_states->publish(msg_base_joint_states);
                pub_odom->publish(calculate_odom(&data));
                // RCLCPP_INFO(this->get_logger(), "Caster rotation %d : %d : %d", msg->fl_caster_rotation_joint, msg->fr_caster_rotation_joint, msg->r_caster_rotation_joint);
                // RCLCPP_INFO(this->get_logger(), "Caster drive %d : %d : %d", msg->fl_caster_drive_joint, msg->fr_caster_drive_joint, msg->r_caster_drive_joint);
            }
            else if (strcmp(zmq_msg_group(&receiveMessage), TOPIC_LIDAR_RANGES) == 0)
            {
                if (bytesReceived != sizeof(LaserScan_t))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("bobik_bridge"), "Invalid message size of LaserScan_t {}", bytesReceived);
                    continue;
                }
                // RCLCPP_INFO(rclcpp::get_logger("bobik_bridge"), "TOPIC_LIDAR_RANGES: %d", sizeof(LaserScan_t));
                void *data_buffer = zmq_msg_data(&receiveMessage);
                LaserScan_t *lidar_ranges = (LaserScan_t *)data_buffer;
                sensor_msgs::msg::LaserScan::SharedPtr msg_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
                msg_scan->header.frame_id = "lidar_link";
                msg_scan->header.stamp = rclcpp::Clock().now();
                msg_scan->angle_min = 0.0;
                msg_scan->angle_max = 2.0 * M_PI;
                msg_scan->angle_increment = (2.0 * M_PI / 360.0);
                msg_scan->range_min = 0.06;
                msg_scan->range_max = 5.0;
                msg_scan->time_increment = lidar_ranges->time_increment / 1e8;
                msg_scan->ranges.resize(360);
                for (int i = 0; i < 360; i++)
                {
                    msg_scan->ranges[i] = (float)(lidar_ranges->data[i]) / 1000.0;
                }
                pub_scan->publish(*msg_scan);
            }
            else if (strcmp(zmq_msg_group(&receiveMessage), TOPIC_IMU9DOF) == 0)
            {
                // https://www.ros.org/reps/rep-0145.html
                // EKF filter takes roll/pitch even when disabled in ekf.yaml. Hardcoding 0 here.
                void *data_buffer = zmq_msg_data(&receiveMessage);
                MsgIMU9DOF_t *imu_data = (MsgIMU9DOF_t *)data_buffer;
                tf2::Quaternion from_q(
                    (double)imu_data->qx / FLOAT_INT16_PRECISION,
                    (double)imu_data->qy / FLOAT_INT16_PRECISION,
                    (double)imu_data->qz / FLOAT_INT16_PRECISION,
                    (double)imu_data->qw / FLOAT_INT16_PRECISION);
                tf2::Matrix3x3 m(from_q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                tf2::Quaternion to_q;
                to_q.setRPY( 0, 0, yaw );
                geometry_msgs::msg::Quaternion imu_orientation;
                imu_orientation.w = to_q.getW();
                imu_orientation.x = 0.0;
                imu_orientation.y = 0.0;
                imu_orientation.z = 0.0; //to_q.getZ();

                // °/sec > rad/sec
                geometry_msgs::msg::Vector3 imu_angular_velocity;
                imu_angular_velocity.x = (double)imu_data->gx * DEG2RAD;
                imu_angular_velocity.y = (double)imu_data->gy * DEG2RAD;
                imu_angular_velocity.z = (double)imu_data->gz * DEG2RAD;

                // LSB*g (1g = 9.80665 m/s^2)> m/s^2
                geometry_msgs::msg::Vector3 imu_linear_acceleration;
                #define ACC_CALIB_X 1.0
                #define ACC_CALIB_Y ACC_CALIB_X
                imu_linear_acceleration.x = -(double)imu_data->ax * G2MS2 - imu_acc_bias_x;
                //+y left    -y right
                imu_linear_acceleration.y = (double)imu_data->ay * G2MS2 - imu_acc_bias_y;  //the lower the more right it drifts
                imu_linear_acceleration.z = (double)imu_data->az * G2MS2 * 1.894;
                if (calibration_cnt < CALIBRATION_FRAMES)  // count to calibration?
                {
                    imu_acc_bias_x_accum += imu_linear_acceleration.x;
                    imu_acc_bias_y_accum += imu_linear_acceleration.y;
                    calibration_cnt++;
                }
                if (calibration_cnt == CALIBRATION_FRAMES)
                {
                    imu_acc_bias_x = imu_acc_bias_x_accum / calibration_cnt;
                    imu_acc_bias_y = imu_acc_bias_y_accum / calibration_cnt;
                    imu_linear_acceleration.x = 0.0;
                    imu_linear_acceleration.y = 0.0;
                    calibration_cnt++;  // do not exec calibration again
                    RCLCPP_INFO(this->get_logger(), "IMU calibration bias: x %.2f , y %.2f", imu_acc_bias_x, imu_acc_bias_y);
                }
                if (abs(imu_linear_acceleration.x) < IMU_STEADY_ACC_TRESHOLD) imu_linear_acceleration.x = 0.0;
                if (abs(imu_linear_acceleration.y) < IMU_STEADY_ACC_TRESHOLD) imu_linear_acceleration.y = 0.0;
                imu_linear_acceleration.x *= ACC_CALIB_X;
                imu_linear_acceleration.y *= ACC_CALIB_Y;

                sensor_msgs::msg::Imu::SharedPtr msg_imu = std::make_shared<sensor_msgs::msg::Imu>();
                msg_imu->header.frame_id = "imu_link";
                msg_imu->header.stamp = rclcpp::Clock().now();
                msg_imu->orientation = imu_orientation;
                msg_imu->orientation_covariance =  boost::assign::list_of(0.01) (0) (0)
                                                                        (0) (0.01)  (0)
                                                                        (0)  (0)  (0.01);
                msg_imu->angular_velocity = imu_angular_velocity;
                msg_imu->angular_velocity_covariance =  boost::assign::list_of(0.03) (0) (0)
                                                                            (0) (0.03)  (0)
                                                                            (0)  (0)  (0.03);
                msg_imu->linear_acceleration = imu_linear_acceleration;
                msg_imu->linear_acceleration_covariance =  boost::assign::list_of(10) (0) (0)
                                                                                (0) (10)  (0)
                                                                                (0)  (0)  (10);
                if (calibration_cnt > CALIBRATION_FRAMES)
                {
                    pub_imu->publish(*msg_imu);
                }

                /*
                sensor_msgs::msg::MagneticField::SharedPtr msg_mag = std::make_shared<sensor_msgs::msg::MagneticField>();
                msg_imu->header.frame_id = "imu_link";
                msg_imu->header.stamp = rclcpp::Clock().now();
                msg_mag->magnetic_field.x = (double)imu_data->mx;
                msg_mag->magnetic_field.y = (double)imu_data->my;
                msg_mag->magnetic_field.z = (double)imu_data->mz;
                msg_mag->magnetic_field_covariance =  boost::assign::list_of(10) (0) (0)
                                                                            (0) (10)  (0)
                                                                            (0)  (0)  (10);
                pub_mag->publish(*msg_mag);
                */
            }
        }

        zmq_msg_close(&receiveMessage);

        status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
}

void BobikBridge::cmd_vel_torobot(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
    geometry_msgs::msg::Vector3 linear = msg->linear;
    geometry_msgs::msg::Vector3 angular = msg->angular;
    MsgCmdVel_t msg_cmd_vel;
    msg_cmd_vel.linear_x = (int16_t)(linear.x * FLOAT_INT16_PRECISION);
    msg_cmd_vel.linear_y = (int16_t)(-linear.y * FLOAT_INT16_PRECISION); // inverting Y axis to match REP-103
    msg_cmd_vel.rotation = (int16_t)(angular.z * FLOAT_INT16_PRECISION);
    // Do not remove this login else rotation will break. Magic.
    uint8_t *x = (uint8_t *)(&msg_cmd_vel);
    RCLCPP_INFO(this->get_logger(), "From ROS2 /cmd_vel: '%02X:%02X:%02X:%02X:%02X:%02X'", x[0], x[1], x[2], x[3], x[4], x[5]);
    send_to_zmq_topic(TOPIC_CMD_VEL, (uint8_t *)(&msg_cmd_vel), 6);
}

nav_msgs::msg::Odometry BobikBridge::calculate_odom(std::vector<int16_t> *data) const
{
    MsgCasterJointStates_t *caster_data = (struct MsgCasterJointStates_t *)data->data();
    // Front Left caster new position
    float Arad = CASTER_UNITS2RAD * caster_data->fl_caster_rotation_joint;
    float Ameters = CASTER_TICKS2METERS * REAL_WORLD_COEF_A * caster_data->fl_caster_drive_joint;
    float Ax = POS_A_x + cos(Arad) * Ameters;
    float Ay = POS_A_y - sin(Arad) * Ameters;
    // Front Right caster new position
    float Brad = CASTER_UNITS2RAD * caster_data->fr_caster_rotation_joint;
    float Bmeters = CASTER_TICKS2METERS * REAL_WORLD_COEF_B * caster_data->fr_caster_drive_joint;
    float Bx = POS_B_x + cos(Brad) * Bmeters;
    float By = POS_B_y - sin(Brad) * Bmeters;
    // Rear caster new position
    float Crad = CASTER_UNITS2RAD * caster_data->r_caster_rotation_joint;
    float Cmeters = CASTER_TICKS2METERS * REAL_WORLD_COEF_C * caster_data->r_caster_drive_joint;
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
    float base_center_y = -(c_center_y + c_center_to_rear_y / 3.0);
    // New base vector forward
    float base_forward_x = -c_center_to_rear_x;
    float base_forward_y = -c_center_to_rear_y; // inverting Y axis to match REP-103
    // Rotation of base in radians
    // float base_pose_rotation = atan2(base_center_y, base_center_x); // assumption: base cannot rotate more than 90 degrees with one frame
    float base_twist_rotation = -atan2(base_forward_y, base_forward_x); // assumption: base cannot rotate more than 90 degrees with one frame

    // persist odom data for next frame
    odom_pose_x += base_center_x;
    odom_pose_y += base_center_y;
    odom_pose_gamma += base_twist_rotation;

    geometry_msgs::msg::Pose odom_pose;
    odom_pose.position.x = odom_pose_x;
    odom_pose.position.y = odom_pose_y;
    odom_pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_pose_gamma);
    odom_pose.orientation = tf2::toMsg(q);

    nav_msgs::msg::Odometry message;
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    rclcpp::Duration delay = rclcpp::Duration(0, MSG_TRANSPORT_DELAY_MS * 1000000);
    message.header.stamp = rclcpp::Clock().now();// - delay;
    message.pose.pose = odom_pose;
    // message.pose.covariance =  boost::assign::list_of(1e-3) (0)    (0)  (0)  (0)  (0)
                                                    //    (0) (1e-3)  (0)  (0)  (0)  (0)
                                                    //    (0)   (0)  (1e6) (0)  (0)  (0)
                                                    //    (0)   (0)   (0) (1e6) (0)  (0)
                                                    //    (0)   (0)   (0)  (0) (1e6) (0)
                                                    //    (0)   (0)   (0)  (0)  (0)  (1e3) ;
    message.twist.twist.linear.x = base_center_x;
    message.twist.twist.linear.y = base_center_y;
    message.twist.twist.linear.z = 0;
    message.twist.twist.angular.x = 0;
    message.twist.twist.angular.y = 0;
    message.twist.twist.angular.z = base_twist_rotation;
    // message.twist.covariance =  boost::assign::list_of(1e-3) (0)    (0)  (0)  (0)  (0)
                                                    //    (0) (1e-3)  (0)  (0)  (0)  (0)
                                                    //    (0)   (0)  (1e6) (0)  (0)  (0)
                                                    //    (0)   (0)   (0) (1e6) (0)  (0)
                                                    //    (0)   (0)   (0)  (0) (1e6) (0)
                                                    //    (0)   (0)   (0)  (0)  (0)  (1e3) ;
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

    if (zmq_connect(radio, "udp://192.168.1.21:7654") != 0)
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
    if (zmq_join(dish, TOPIC_IMU9DOF) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("bobik_bridge"), "Could not subscribe to: %s", TOPIC_IMU9DOF);
        return EXIT_FAILURE;
    }
    if (zmq_join(dish, TOPIC_LIDAR_RANGES) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("bobik_bridge"), "Could not subscribe to: %s", TOPIC_LIDAR_RANGES);
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
