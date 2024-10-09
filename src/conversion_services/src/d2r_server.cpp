// src/d2r_server.cpp
#include "ros/ros.h"
#include "conversion_services/d2r.h"
// Function to convert degrees to radians
double d2r(double degrees) {
    return degrees * (M_PI / 180.0);
}

bool handleD2R(conversion_services::d2r::Request &req,
               conversion_services::d2r::Response &res) {
    res.radians = d2r(req.degrees);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "d2r_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("d2r", handleD2R);
    ROS_INFO("Ready to convert degrees to radians.");
    ros::spin();

    return 0;
}

