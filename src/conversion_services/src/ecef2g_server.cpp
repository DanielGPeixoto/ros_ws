// src/ecef2g_server.cpp
#include "ros/ros.h"
#include "conversion_services/ecef2g.h"
// Function to convert ECEF (x, y, z) to LLA (latitude, longitude, altitude)
void ecef2g(double x, double y, double z, double& latitude, double& longitude, double& altitude) {
    const double a = 6378137.0;  // WGS84 semi-major axis
    const double e2 = 0.00669437999014;  // Square of eccentricity
    const double ep = sqrt((a * a) / (1 - e2)); // Prime vertical radius of curvature

    longitude = atan2(y, x);
    double p = sqrt(x * x + y * y);
    latitude = atan2(z, p * (1 - e2)); // Initial latitude

    double N;
    double prev_latitude = 0.0;
    altitude = 0.0;

    // Iteratively solve for latitude
    while (fabs(latitude - prev_latitude) > 1e-10) {
        prev_latitude = latitude;
        N = a / sqrt(1 - e2 * sin(latitude) * sin(latitude));
        altitude = p / cos(latitude) - N;
        latitude = atan2(z, p * (1 - e2 * N / (N + altitude)));
    }

    latitude = latitude * (180.0 / M_PI);
    longitude = longitude * (180.0 / M_PI);
}

bool handleECEF2G(conversion_services::ecef2g::Request &req,
                  conversion_services::ecef2g::Response &res) {
    ecef2g(req.x, req.y, req.z, res.latitude, res.longitude, res.altitude);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ecef2g_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("ecef2g", handleECEF2G);
    ROS_INFO("Ready to convert ECEF to LLA.");
    ros::spin();

    return 0;
}

