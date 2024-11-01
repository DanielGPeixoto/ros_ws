#include <cstdlib>
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "tele_op_keyboard_node");
    ros::NodeHandle nh;

    ROS_INFO("Starting tele_op_keyboard node...");
    system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py");

    return 0;
}

