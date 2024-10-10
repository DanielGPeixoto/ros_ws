#include "ros/ros.h"
#include "conversion_services/d2r.h"
double d2r(double degrees) {
    return (degrees / 180.0) * M_PI;
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
    ROS_INFO("Convert degrees to radians.");
    ros::spin();

    return 0;
}

