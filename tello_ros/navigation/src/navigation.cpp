#include "navigation/navigation.hpp"

Navigation::Navigation() : Node("navigation")
{
    index_ = 0;

    declare_parameter<std::string>("topics.path_topic_name", "path_topic_name_");

    path_topic_name_ = get_parameter("topics.path_topic_name").as_string();
 

    path_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>("poses3d", 1, std::bind(&Navigation::pathCallback, this, std::placeholders::_1));
}


void Navigation::pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Path Callback !!!");
    // following_path_ = std::move(msg);
    // path_received_ = true;
    RCLCPP_ERROR(this->get_logger(), "PATH RECEIVED !!!");

}

