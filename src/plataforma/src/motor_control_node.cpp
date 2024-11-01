#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <wiringPi.h>
#include <softPwm.h>

#define M1 28
#define M2 29
#define PWMA 25

#define M3 22
#define M4 23
#define PWMB 26

void setupPins() {
    wiringPiSetup();
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(M3, OUTPUT);
    pinMode(M4, OUTPUT);

    softPwmCreate(PWMA, 0, 255);
    softPwmCreate(PWMB, 0, 255);
}

void sendPWMToMotors(float left_pwm, float right_pwm) {

    if (left_pwm > 0) {
        digitalWrite(M1, LOW);
        digitalWrite(M2, HIGH);
    } else {
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
        left_pwm = -left_pwm;  
    }

    if (right_pwm > 0) {
        digitalWrite(M3, LOW);
        digitalWrite(M4, HIGH);
    } else {
        digitalWrite(M3, HIGH);
        digitalWrite(M4, LOW);
        right_pwm = -right_pwm;  
    }

    softPwmWrite(PWMA, static_cast<int>(left_pwm));
    softPwmWrite(PWMB, static_cast<int>(right_pwm));
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    float max_linear_speed = 1.0;
    float max_angular_speed = 1.0;

    float max_pwm = 255.0;
    float left_pwm, right_pwm;

    left_pwm = (msg->linear.x - msg->angular.z * 0.5) * (max_pwm / max_linear_speed);
    right_pwm = (msg->linear.x + msg->angular.z * 0.5) * (max_pwm / max_linear_speed);

    left_pwm = std::max(std::min(left_pwm, max_pwm), -max_pwm);
    right_pwm = std::max(std::min(right_pwm, max_pwm), -max_pwm);

    sendPWMToMotors(left_pwm, right_pwm);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_control_node");
    ros::NodeHandle n;

    setupPins();

    ros::Subscriber sub = n.subscribe("/cmd_vel", 10, cmdVelCallback);

    ROS_INFO("Motor control node started, listening to /cmd_vel topic.");
    ros::spin();

    return 0;
}

