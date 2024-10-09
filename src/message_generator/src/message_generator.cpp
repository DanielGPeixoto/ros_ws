
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "message_generator/sensor_messages.h"
#include "sensor_msgs/RelativeHumidity.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/NavSatFix.h"
#include <ctime>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<message_generator::sensor_messages>("chatter", 1000);

    ros::Rate loop_rate(10);

    std::srand(std::time(0));  // Seed for random generator

    while (ros::ok())
    {
        message_generator::sensor_messages msg;

        msg.header.stamp = ros::Time::now();

        // Generate random sensor data
        msg.radiation_data = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 1.0;
        msg.humidity_data.header.stamp = ros::Time::now();
        msg.humidity_data.relative_humidity = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 100.0; 
        msg.humidity_data.variance = 0;

        msg.temperature_data.header.stamp = ros::Time::now();
        msg.temperature_data.temperature = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 30.0;
        msg.temperature_data.variance = 0;

        msg.gps_data.header.stamp = ros::Time::now();  // Set GPS timestamp
        msg.gps_data.latitude = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 180.0 - 90.0;  // Latitude between -90 and 90
        msg.gps_data.longitude = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 360.0 - 180.0;  // Longitude between -180 and 180
        msg.gps_data.altitude = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 1000.0;  // Altitude between 0 and 1000

        // Log the values with timestamps
        ROS_INFO("Timestamp: %f, Radiation: %f, HumidityTimeStamp: %f, Humidity: %f, TemperatureTimeStamp: %f, Temperature: %f, GPSTimeStamp: %f, Latitude: %f, Longitude: %f, Altitude: %f",
                 msg.header.stamp.toSec(),
                 msg.radiation_data,
                 msg.humidity_data.header.stamp.toSec(),
                 msg.humidity_data.relative_humidity,
                 msg.temperature_data.header.stamp.toSec(),
                 msg.temperature_data.temperature,
                 msg.gps_data.header.stamp.toSec(),
                 msg.gps_data.latitude,
                 msg.gps_data.longitude,
                 msg.gps_data.altitude);

        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

