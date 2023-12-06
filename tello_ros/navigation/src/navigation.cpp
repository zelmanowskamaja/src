#include "navigation/navigation.hpp"
// #include <tf2_eigen/tf2_eigen.hpp>

Navigation::Navigation() : Node("navigation_node")
{
    index_ = 0;

    declare_parameter<std::string>("topics.path_topic_name", "path_topic_name_");

    path_topic_name_ = get_parameter("topics.path_topic_name").as_string();
 

    path_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>("poses3d", 1, std::bind(&Navigation::pathCallback, this, std::placeholders::_1));
    flight_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 1, std::bind(&Navigation::flight_data_callback, this, std::placeholders::_1));
    

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("drone1/cmd_vel", 1);
}


void Navigation::pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    int al = end(msg->poses)-begin(msg->poses); //length calculation
    RCLCPP_ERROR_STREAM(this->get_logger(),"goal number  = :" << this->start << "-----------------");
    
    if(this->start >= al){
        RCLCPP_ERROR_STREAM(this->get_logger(),"finished");
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.angular.z = 0.0;
        cmd_vel_pub_->publish(twist);
        return;
    }
    // RCLCPP_INFO(this->get_logger(), "Path Callback !!!");
    // following_path_ = std::move(msg);
    // path_received_ = true;
    RCLCPP_ERROR(this->get_logger(), "PATH RECEIVED !!!"); 
    // relative pose to target 
    double pose_x = msg->poses[start].position.x - this->x_;
    double pose_y = msg->poses[start].position.y - this->y_;
    
    tf2::Quaternion q_ = tf2::Quaternion(msg->poses[start].orientation.x, msg->poses[start].orientation.y, msg->poses[start].orientation.z, msg->poses[start].orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_).getRPY(roll, pitch, yaw, 1);
    // RCLCPP_ERROR_STREAM(this->get_logger(),"yaw goal = :" << yaw);
    // RCLCPP_ERROR_STREAM(this->get_logger(),"x goal = " << msg->poses[start].position.x);
    // RCLCPP_ERROR_STREAM(this->get_logger(),"y goal = " << msg->poses[start].position.y);

    double pose_yaw = yaw - this->yaw_;

    // obecna poza 
    if (this->odom_started){
        tf2::Quaternion this_q_ = tf2::Quaternion(this->qx_, this->qy_, this->qz_, this->qw_);
        double this_roll, this_pitch, this_yaw;
        tf2::Matrix3x3(this_q_).getRPY(this_roll, this_pitch, this_yaw, 1);
        // RCLCPP_ERROR_STREAM(this->get_logger(),"this_yaw = :" << this_yaw);
        // RCLCPP_ERROR_STREAM(this->get_logger(),"this_x = :" << this->x_);
        // RCLCPP_ERROR_STREAM(this->get_logger(),"this_y = :" << this->y_);
        // RCLCPP_ERROR_STREAM(this->get_logger(),"this_z = :" << this->z_);
    }

    RCLCPP_ERROR_STREAM(this->get_logger(),"remaining distance x = :" << pose_x);
    RCLCPP_ERROR_STREAM(this->get_logger(),"remaining distance y = :" << pose_y);


    double k = 0.7;
    double v = k * sqrt(pow(pose_x,2)+ pow(pose_y,2));

    // zbiegajace pole
    double x1 = - pose_x - v;//- 0.70* norm(q.x(), q.y());
    double y1 = - pose_y;

    // proste pole 
    double x2 = - pose_x ;
    double y2 = 0.0;

    double scaling_factor = 2.0;

    double scale =  v*scaling_factor;
    if(scale < 0){scale =0;}
    if(scale > 1){scale =1;}
    // im mniejsza odleglosc tym bardziej proste pole 
    // in wiekszy kat tym badziej zakzywione pole <-----------
    // RCLCPP_INFO_STREAM(this->get_logger(), "scale: "<< scale );

    double d_norm = (x1*cos(pose_yaw)+y1*sin(pose_yaw))/sqrt(x1*x1+y1*y1);
    double dhx = -cos(pose_yaw)-k*d_norm;
    double dhy = -sin(pose_yaw);

    double msg_h_x = x1*scale + x2*(1.0 -scale);
    double msg_h_y = y1*scale + y2*(1.0 -scale);
    double msg_dh_x = (scale - 1)*cos(pose_yaw) +pose_x*scaling_factor*d_norm + scale*dhx + x1*scaling_factor*d_norm;
    double msg_dh_y = scale*dhy + y1*scaling_factor*d_norm;
    //vector sterujacy potem przepisuje,y na predkosci 

    // Eigen::Vector2d vec_h(msg_h_x, msg_h_x);//////////////////////////////////////////////////////////////////
    // vec_h.normalize();
    // double h_norm = vec_h.norm();
    double projection = msg_h_x*cos(pose_yaw) + msg_h_y*sin(pose_yaw); 
    
  
    projection = std::max(projection, 0.1);
    double ref_lin_vel_x = std::max(0.01, 0.25*projection);     
    double ref_lin_vel_y = std::max(0.01, 0.25*projection);     
    ref_lin_vel_x = pose_x;
    ref_lin_vel_y = pose_y;
    double max_vel_lin = 0.05;
    if(ref_lin_vel_x > max_vel_lin)
      ref_lin_vel_x = max_vel_lin;
    if(ref_lin_vel_x < -max_vel_lin)
      ref_lin_vel_x = -max_vel_lin;  
    if(ref_lin_vel_y > max_vel_lin)
      ref_lin_vel_y = max_vel_lin;
    if(ref_lin_vel_y < -max_vel_lin)
      ref_lin_vel_y = -max_vel_lin;  

    
    //////////
    
    double k_a_modulated = 2.0;
    // double theta_a = (atan2(vec_h.y(), vec_h.x()));

    double dtheta_a = ref_lin_vel_x*(msg_dh_y*msg_h_x - msg_h_y*msg_dh_x)/(std::sqrt(msg_h_x) + std::sqrt(msg_h_y));      


    double cmd_ang_vel = k_a_modulated*pose_yaw + dtheta_a; 
    cmd_ang_vel = 0.0; 

    RCLCPP_WARN_STREAM(this->get_logger(), "ang vel: " << cmd_ang_vel);  

    double max_vel = 0.05;
    if(cmd_ang_vel > max_vel)
      cmd_ang_vel = max_vel;
    if(cmd_ang_vel < -max_vel)
      cmd_ang_vel = -max_vel;      
    /////////////////////////////////////////



    if(pose_x < 0.02 && pose_x > - 0.02 && pose_y < 0.02 && pose_y > -0.02){
        this->start++;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = ref_lin_vel_x;
    twist.linear.y = ref_lin_vel_y;
    twist.angular.z = cmd_ang_vel;
    cmd_vel_pub_->publish(twist);
    RCLCPP_INFO_STREAM(this->get_logger(), "publishing drone cmd_vel " );

}
void Navigation::flight_data_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
    // calculate pose base on flight data, starts in 0,0,0
    // RCLCPP_ERROR(this->get_logger(), "flight data callback !!!");
    // RCLCPP_ERROR_STREAM(this->get_logger(),"msg = :" << msg->pose);

    rclcpp::Time now = this->get_clock()->now();
    // double time_diff = (now - rclcpp::Time(msg->header.stamp)).nanoseconds()/1e9;

    // this->roll_= msg->roll;
    // this->pitch_= msg->pitch;
    // this->yaw_= msg->yaw;
    this->x_ = msg->pose[1].position.x;
    this->y_ = msg->pose[1].position.y;
    this->z_ = msg->pose[1].position.z;
    // RCLCPP_ERROR_STREAM(this->get_logger(),"msg = :" << x_ << y_ <<z_);


    // this->x_ += msg->vgx*cos(this->yaw_)*time_diff - msg->vgy*sin(this->yaw_)*time_diff;
    // this->y_ += msg->vgx*sin(this->yaw_)*time_diff + msg->vgy*cos(this->yaw_)*time_diff;
    this->qx_ = msg->pose[1].orientation.x;
    this->qy_ = msg->pose[1].orientation.y;
    this->qz_ = msg->pose[1].orientation.z;
    this->qw_ = msg->pose[1].orientation.w;

    this->odom_started = true;


}
// odometry callback{
//     code here 
// }
