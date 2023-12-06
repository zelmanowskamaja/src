#ifndef NAVIGATION_NODE_HPP
#define NAVIGATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tello_msgs/msg/flight_data.hpp>
#include <gazebo_msgs/msg/model_states.hpp>


#include "tf2/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cmath>

using namespace std::chrono_literals;

class Navigation : public rclcpp::Node
{
    public:
        Navigation();
    
    private:
        // node parameters
        std::string path_topic_name_;

        bool odom_started = false;
        int start = 0;


        double x_ = 0;
        double y_ = 0;
        double z_ = 0;

        double roll_;
        double pitch_;
        double yaw_ = 0;

        double qx_;
        double qy_;
        double qz_;
        double qw_;

        int index_;

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_subscriber_;
        rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr flight_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        void flight_data_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
};

#endif // NAVIGATION_NODE_HPP