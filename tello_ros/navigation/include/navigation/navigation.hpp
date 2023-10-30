#ifndef NAVIGATION_NODE_HPP
#define NAVIGATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>


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

        double x_;
        double y_;
        double z_;

        double roll_;
        double pitch_;
        double yaw_;

        int index_;

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_subscriber_;
        void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
};

#endif // NAVIGATION_NODE_HPP