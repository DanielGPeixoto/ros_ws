#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.h"
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float64.h"

using namespace cv;

ros::Publisher pub_image_edges;
ros::Publisher pub_image_result;
ros::Publisher pub_error;
ros::Publisher pub_detected_colors;

std::string last_detected_color = "";


void image_callback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now(); 
    header.frame_id = "camera_frame";

    Mat image_decode = cv::imdecode(msg->data, cv::IMREAD_COLOR);
    Rect Rec(0, 154, 410, 154); 
    Mat image_decode_rec = image_decode(Rec);

    Mat image_gaussian_blur;
    GaussianBlur(image_decode_rec, image_gaussian_blur, Size(5, 5), 1.5);

    Mat img_RGB;
    cvtColor(image_gaussian_blur, img_RGB, COLOR_BGR2RGB);
    Mat img_hsv;
    cvtColor(img_RGB, img_hsv, COLOR_RGB2HSV);

    Mat edges;
    inRange(img_RGB, Scalar(0, 0, 0), Scalar(60, 60, 60), edges); 
    Canny(edges, edges, 50, 200); 

    Mat green_mask, blue_mask, red_mask1, red_mask2, red_mask;
    inRange(img_hsv, Scalar(30, 50, 50), Scalar(90, 255, 255), green_mask);
    inRange(img_hsv, Scalar(100, 150, 0), Scalar(140, 255, 255), blue_mask);
    inRange(img_hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), red_mask1);
    inRange(img_hsv, Scalar(160, 100, 100), Scalar(180, 255, 255), red_mask2);
    red_mask = red_mask1 | red_mask2; 

    Mat result = green_mask | blue_mask | red_mask;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(result, result, MORPH_CLOSE, kernel);
    morphologyEx(result, result, MORPH_OPEN, kernel);

    std::vector<std::vector<Point>> contours;
    findContours(result, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    std::string detected_colors = "";
    const int MIN_AREA_THRESHOLD = 100;

    for (const auto& contour : contours) {
        double area = contourArea(contour);
        if (area > MIN_AREA_THRESHOLD) {
            if (countNonZero(green_mask) > 0) detected_colors = "Green ";
            if (countNonZero(blue_mask) > 0) detected_colors = "Blue ";
            if (countNonZero(red_mask) > 0) detected_colors = "Red ";
        }
    }

    if (detected_colors != last_detected_color) {
        last_detected_color = detected_colors;

        std_msgs::String color_msg;
        color_msg.data = detected_colors;
        pub_detected_colors.publish(color_msg);

        ROS_INFO_STREAM("Detected Color Changed to: " << detected_colors);
    }

    if (detected_colors.empty() && !last_detected_color.empty()) {
        last_detected_color = "";

        std_msgs::String color_msg;
        color_msg.data = "None";
        pub_detected_colors.publish(color_msg);

        ROS_INFO("No color detected.");
    }

    int num = 0;
    unsigned int pontos_linha_100[2];
    int linha = 100;

    for (int coluna = 20; coluna < 390; coluna++) {
        if (edges.at<uchar>(linha, coluna) != 0 && num == 0) {
            pontos_linha_100[num] = coluna;
            num++;
        }
        if (edges.at<uchar>(linha, coluna) != 0 && num == 1 && coluna > (pontos_linha_100[num - 1] + 20)) {
            pontos_linha_100[num] = coluna;
            num++;
        }
    }

    if (num == 2) { 
        int line_center = (pontos_linha_100[0] + pontos_linha_100[1]) / 2;
        int image_center = edges.cols / 2;
        double error = image_center - line_center;

        std_msgs::Float64 error_msg;
        error_msg.data = error;
        pub_error.publish(error_msg);
    }

    sensor_msgs::Image img_msg_edges; 
    Mat edges_bgr;
    cvtColor(edges, edges_bgr, COLOR_GRAY2BGR);
    cv_bridge::CvImage img_bridge_edges(header, sensor_msgs::image_encodings::BGR8, edges_bgr);
    img_bridge_edges.toImageMsg(img_msg_edges);
    pub_image_edges.publish(img_msg_edges);

    ROS_INFO_STREAM("Detected Colors: " << detected_colors);

    Mat result_bgr;
    cvtColor(result, result_bgr, COLOR_GRAY2BGR);
    cv_bridge::CvImage img_bridge_result(header, sensor_msgs::image_encodings::BGR8, result_bgr);
    sensor_msgs::Image img_msg_result;
    img_bridge_result.toImageMsg(img_msg_result);
    pub_image_result.publish(img_msg_result);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processing");
    ros::NodeHandle n;
    ros::Subscriber sub_Cam = n.subscribe<sensor_msgs::CompressedImage>("/raspicam_node/image/compressed", 1, image_callback);
    pub_image_edges = n.advertise<sensor_msgs::Image>("/Publisher_Image", 1);
    pub_image_result = n.advertise<sensor_msgs::Image>("/color_detection_image", 1);
    pub_detected_colors = n.advertise<std_msgs::String>("/detected_colors", 1);
    pub_error = n.advertise<std_msgs::Float64>("/line_error", 1);

    ros::spin();
    return 0;
}

