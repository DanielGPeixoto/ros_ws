#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_srvs/SetBool.h" 
#include <chrono>

// batteries @3.90 double Kp = 0.0004;  double Kd = 0.00008;     motor_cmd.linear.x = -0.06;  // Constant forward speed
// batteries @4.08V double Kp = 0.0008;  double Kd = 0.0003;     motor_cmd.linear.x = -0.08;  // Constant forward speed

double Kp = 0.001;
double Ki = 0.0;
double Kd = 0.0004;

double previous_error = 0.0;
double integral = 0.0;
ros::Time last_time;
ros::Publisher motor_cmd_pub;

bool pid_enabled = true;

bool togglePID(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    pid_enabled = req.data;
    res.success = true;
    res.message = pid_enabled ? "PID enabled" : "PID disabled";
    return true;
}

void errorCallback(const std_msgs::Float64::ConstPtr& error_msg) {

    if (!pid_enabled) {
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        motor_cmd_pub.publish(stop_cmd);
        return;
    }
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    if (dt <= 0) {
        return;
    }

    double error = error_msg->data;

    double proportional = Kp * error;

    integral += error * dt;
    double integral_term = Ki * integral;

    double derivative = (error - previous_error) / dt;
    double derivative_term = Kd * derivative;

    double control_signal = proportional + integral_term + derivative_term;

    previous_error = error;
    last_time = current_time;

    ROS_INFO("Error: %f, P-Term: %f, I-Term: %f, D-Term: %f, Control: %f",
             error, proportional, integral_term, derivative_term, control_signal);

    geometry_msgs::Twist motor_cmd;
    motor_cmd.linear.x = -0.10;
    motor_cmd.angular.z = control_signal;
    motor_cmd_pub.publish(motor_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_controller_node");
    ros::NodeHandle nh;

    motor_cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber error_sub = nh.subscribe("line_error", 10, errorCallback);
    ros::ServiceServer service = nh.advertiseService("toggle_pid_control", togglePID);

    last_time = ros::Time::now();

    ROS_INFO("Line Controller Node started...");
    ros::spin();

    return 0;
}

