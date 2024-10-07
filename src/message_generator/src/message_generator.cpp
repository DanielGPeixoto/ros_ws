#include "ros/ros.h"
#include "std_msgs/String.h"
#include "message_generator/sensor_messages.h"
#include "sensor_msgs/RelativeHumidity.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/NavSatFix.h"
#include <random> 
#include <ctime> 
#include <algorithm> 

float generateRandomValue(float mean, float variance) {
    static std::default_random_engine generator(std::time(0)); 
    std::normal_distribution<float> distribution(mean, std::sqrt(variance)); 
    
    float random_value = distribution(generator);
    
    if (mean == 50.0) { 
        return std::max(0.0f, std::min(100.0f, random_value));
    }
    return random_value; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<message_generator::sensor_messages>("chatter", 1000);

    ros::Rate loop_rate(10);

    std::srand(std::time(0));

    float humidity_mean = 50.0;  
    float humidity_variance = 70.0; 

    float temperature_mean = 25.0;  
    float temperature_variance = 20.0; 

    float radiation_mean = 0.1;  
    float radiation_variance = 0.05; 

    while (ros::ok())
    {
        message_generator::sensor_messages msg;

        msg.header.stamp = ros::Time::now();

        msg.humidity_data.relative_humidity = generateRandomValue(humidity_mean, humidity_variance);
        msg.humidity_data.variance = humidity_variance;  

        msg.temperature_data.temperature = generateRandomValue(temperature_mean, temperature_variance);
        msg.temperature_data.variance = temperature_variance;  

        msg.radiation_data = generateRandomValue(radiation_mean, radiation_variance);

        msg.gps_data.latitude = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 180.0 - 90.0; 
        msg.gps_data.longitude = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 360.0 - 180.0;
        msg.gps_data.altitude = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 1000.0; 

        ROS_INFO("Timestamp: %f, Humidity: %f, Temperature: %f, Radiation: %f, Latitude: %f, Longitude: %f, Altitude: %f",
                 msg.header.stamp.toSec(),
                 msg.humidity_data.relative_humidity,
                 msg.temperature_data.temperature,
                 msg.radiation_data,
                 msg.gps_data.latitude,
                 msg.gps_data.longitude,
                 msg.gps_data.altitude);

        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

