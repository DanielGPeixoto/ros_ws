#include "ros/ros.h"
#include "conversion_services/g2ecef.h"
void g2ecef(double latitude, double longitude, double altitude, double& x, double& y, double& z) {
    const double a = 6378137.0;
    const double e2 = 0.00669437999014;

    double radLat = latitude * (M_PI / 180.0);
    double radLon = longitude * (M_PI / 180.0);
    double N = a / sqrt(1 - e2 * sin(radLat) * sin(radLat));

    x = (N + altitude) * cos(radLat) * cos(radLon);
    y = (N + altitude) * cos(radLat) * sin(radLon);
    z = ((1 - e2) * N + altitude) * sin(radLat);
}

bool handleG2ECEF(conversion_services::g2ecef::Request &req,
                  conversion_services::g2ecef::Response &res) {
    g2ecef(req.latitude, req.longitude, req.altitude, res.x, res.y, res.z);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "g2ecef_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("g2ecef", handleG2ECEF);
    ROS_INFO("Convert LLA to ECEF.");
    ros::spin();

    return 0;
}

