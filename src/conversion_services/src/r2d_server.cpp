#include "ros/ros.h"
#include "conversion_services/r2d.h"

double r2d(double radians) {
    return (radians / M_PI) * 180.0;
}

bool handleR2D(conversion_services::r2d::Request &req,
               conversion_services::r2d::Response &res) {
    res.degrees = r2d(req.radians);  
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "r2d_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("r2d", handleR2D);
    ROS_INFO("Convert radians to degrees.");
    ros::spin();

    return 0;
}

