
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <chrono>  // Include for time tracking

// Initialize PID parameters with default values
// baterias @3.90 double Kp = 0.0004;  double Kd = 0.00008;     motor_cmd.linear.x = -0.06;  // Constant forward speed
//
// baterias @4.08V double Kp = 0.0008;  double Kd = 0.0003;     motor_cmd.linear.x = -0.08;  // Constant forward speed



double Kp = 0.0008;
double Ki = 0.0;
double Kd = 0.0003;

double previous_error = 0.0;
double integral = 0.0;
ros::Time last_time;  // Variable to hold the last time when the error was calculated
ros::Publisher motor_cmd_pub;

void errorCallback(const std_msgs::Float64::ConstPtr& error_msg) {
    // Get current time
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();  // Calculate time difference in seconds

    if (dt <= 0) {  // Prevent division by zero
        return;  // If no time has passed, do not proceed
    }

    double error = error_msg->data;

    // PID calculations
    double proportional = Kp * error;

    integral += error * dt;  // Update integral term with time step
    double integral_term = Ki * integral;

    double derivative = (error - previous_error) / dt;  // Calculate derivative term
    double derivative_term = Kd * derivative;

    double control_signal = proportional + integral_term + derivative_term;

    // Update previous error and last time for the next callback
    previous_error = error;
    last_time = current_time;

    // Print debug information
    ROS_INFO("Error: %f, P-Term: %f, I-Term: %f, D-Term: %f, Control: %f",
             error, proportional, integral_term, derivative_term, control_signal);

    // Send control command
    geometry_msgs::Twist motor_cmd;
    motor_cmd.linear.x = -0.08;  // Constant forward speed
    motor_cmd.angular.z = control_signal;
    motor_cmd_pub.publish(motor_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_controller_node");
    ros::NodeHandle nh;

    // Publisher and subscriber setup
    motor_cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber error_sub = nh.subscribe("line_error", 10, errorCallback);

    // Initialize last_time to the current time
    last_time = ros::Time::now();

    ROS_INFO("Line Controller Node started...");
    ros::spin();

    return 0;
}

