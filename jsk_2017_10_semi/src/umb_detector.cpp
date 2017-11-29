#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"

#include "std_msgs/String.h"
#include "jsk_2017_10_semi/umb_pos.h"
#include <sstream>
#include <iostream>
#include <string>

class UmbDetector
{
private:
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO("Received image");
        cv::Mat in_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;

        cv::Mat gray_img;
        cv::cvtColor(in_img, gray_img, CV_BGR2GRAY);

        cv::Mat result_img;

        cv::matchTemplate(in_img, tmp_img, result_img, CV_TM_CCOEFF_NORMED);

        cv::Rect roi_rect(0, 0, tmp_img.cols, tmp_img.rows);
        cv::Point max_pt;
        double maxVal;
        cv::minMaxLoc(result_img, NULL, &maxVal, NULL, &max_pt);
        roi_rect.x = max_pt.x;
        roi_rect.y = max_pt.y;

        if (maxVal > match_coeff_thresh) {
            //add rectangle RED
            cv::rectangle(in_img, roi_rect, cv::Scalar(0, 0, 255, 3));
        } else {
            //add rectangle BLUE
            cv::rectangle(in_img, roi_rect, cv::Scalar(255, 0, 0, 3));
        }

        std::stringstream stream;
        stream << maxVal;
        cv::putText(in_img, stream.str(), max_pt, 1, 3.0, cv::Scalar(100, 255, 255, 3));

        cv::imshow("in_img", in_img);
        cv::waitKey(1);
    }

public:
    UmbDetector(ros::NodeHandle nh = ros::NodeHandle()) : it_(nh), node_handle(ros::NodeHandle())
    {
        //img_sub_ = it_.subscribe("/head_camera/rgb/image_raw", 3, &UmbDetector::imageCallBack, this);
        img_sub_ = it_.subscribe("image", 3, &UmbDetector::imageCallBack, this);
        cv::namedWindow("Fast", 1);

        chatter_pub = node_handle.advertise<jsk_2017_10_semi::umb_pos>("umb_pos", 1000);

        tmp_img = cv::imread("/home/kogatti/semi_ws/src/jsk_demos/jsk_2017_10_semi/picture/umb_handle.png", 1);
        if (tmp_img.empty()){
            std::cout << "couldn't read the image. ./../picture/umb_handle.png" << std::endl;
            return;
        }
    }

    void publishDetectUmb(float x, float y, float z) const
    {
        jsk_2017_10_semi::umb_pos msg;
        msg.x = x;
        msg.y = y;
        msg.z = z;

        chatter_pub.publish(msg);
    }

private:
    image_transport::Subscriber img_sub_;
    image_transport::ImageTransport it_;
    ros::NodeHandle node_handle;
    ros::Publisher chatter_pub;

    cv::Mat tmp_img;

    double umb_length = 800.0;
    double ideal_height = 20.0;
    double ideal_dist = 1000.0;
    double match_coeff_thresh = 0.65;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "umb_detector_node");
    UmbDetector mcp;
    ros::spin();
}
