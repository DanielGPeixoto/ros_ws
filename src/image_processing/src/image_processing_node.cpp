#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Header.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.h"
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float64.h"

using namespace cv;

ros::Publisher pub_image;
ros::Publisher pub_error;

void image_callback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    Mat image_decode = cv::imdecode(msg->data, cv::IMREAD_COLOR);
    Rect Rec(0, 154, 410, 154);
    Mat image_decode_rec = image_decode(Rec);

    Mat image_gaussian_blur;
    GaussianBlur(image_decode_rec, image_gaussian_blur, Size(5,5), 1.5);

    Mat img_RGB;
    cvtColor(image_gaussian_blur, img_RGB, COLOR_BGR2RGB);
    inRange(img_RGB, Scalar(0, 0, 0), Scalar(60, 60, 60), img_RGB);

    Mat edges;
    Canny(img_RGB, edges, 50, 200);

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

    int line_center = (pontos_linha_100[0] + pontos_linha_100[1]) / 2;
    int image_center = edges.cols / 2;
    double error = image_center - line_center;

    std_msgs::Float64 error_msg;
    error_msg.data = error;
    pub_error.publish(error_msg);

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, edges);
    img_bridge.toImageMsg(img_msg);
    pub_image.publish(img_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processing");
    ros::NodeHandle n;
    ros::Subscriber sub_Cam = n.subscribe<sensor_msgs::CompressedImage>("/raspicam_node/image/compressed", 1, image_callback);
    pub_image = n.advertise<sensor_msgs::Image>("/Publisher_Image", 1);
    pub_error = n.advertise<std_msgs::Float64>("/line_error", 1);

    ros::spin();
    return 0;
}

