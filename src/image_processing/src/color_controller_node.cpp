#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include <chrono>

ros::ServiceClient toggle_pid_client;
ros::Time last_red_detection_time;
ros::Time last_green_detection_time;
ros::Time last_blue_detection_time;
double color_cooldown = 1.0;

void colorCallback(const std_msgs::String::ConstPtr& color_msg) {
    std::string detected_color = color_msg->data;
    ros::Time current_time = ros::Time::now();
    ROS_INFO_STREAM("Colors: " << detected_color);

     if (detected_color.find("Red") != std::string::npos && (current_time - last_red_detection_time).toSec() >= color_cooldown) {
        ROS_INFO("Red color detected! Pausing line following...");

        last_red_detection_time = current_time;

        std_srvs::SetBool srv;
        srv.request.data = false;  

        if (toggle_pid_client.call(srv)) {
            ROS_INFO("Line following paused successfully.");

            ros::Duration(1.0).sleep();

            srv.request.data = true;  
            if (toggle_pid_client.call(srv)) {
                ROS_INFO("Line following resumed.");
            } else {
                ROS_ERROR("Failed to resume line following.");
            }
        } else {
            ROS_ERROR("Failed to pause line following.");
        }
    }

     if (detected_color.find("Blue") != std::string::npos && (current_time - last_blue_detection_time).toSec() >= color_cooldown) {
        ROS_INFO("Blue color detected! Pausing line following...");

        last_red_detection_time = current_time;

        std_srvs::SetBool srv;
        srv.request.data = false;  

        if (toggle_pid_client.call(srv)) {
            ROS_INFO("Line following paused successfully.");

            ros::Duration(2.0).sleep();

            srv.request.data = true;  
            if (toggle_pid_client.call(srv)) {
                ROS_INFO("Line following resumed.");
            } else {
                ROS_ERROR("Failed to resume line following.");
            }
        } else {
            ROS_ERROR("Failed to pause line following.");
        }
    }

     if (detected_color.find("Green") != std::string::npos && (current_time - last_green_detection_time).toSec() >= color_cooldown) {
        ROS_INFO("Green color detected! Pausing line following...");

        last_red_detection_time = current_time;

        std_srvs::SetBool srv;
        srv.request.data = false;  

        if (toggle_pid_client.call(srv)) {
            ROS_INFO("Line following paused successfully.");

            ros::Duration(3.0).sleep();

            srv.request.data = true;  
            if (toggle_pid_client.call(srv)) {
                ROS_INFO("Line following resumed.");
            } else {
                ROS_ERROR("Failed to resume line following.");
            }
        } else {
            ROS_ERROR("Failed to pause line following.");
        }
    }


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "color_controller_node");
    ros::NodeHandle nh;

    toggle_pid_client = nh.serviceClient<std_srvs::SetBool>("toggle_pid_control");

    ros::Subscriber color_sub = nh.subscribe("detected_colors", 10, colorCallback);

    last_red_detection_time = ros::Time::now() - ros::Duration(color_cooldown);

    ROS_INFO("Color Controller Node started...");
    ros::spin();

    return 0;
}

