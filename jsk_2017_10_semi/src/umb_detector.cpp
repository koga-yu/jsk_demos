#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"

#include "std_msgs/String.h"
#include <sstream>

class UmbDetector
{
private:
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO("Received image");
        cv::Mat in_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;

        cv::Mat grayImage, binImage;
        cv::cvtColor(in_img, grayImage, CV_BGR2GRAY);
        cv::threshold(grayImage, binImage, 128.0, 255.0, CV_THRESH_OTSU);
        cv::imshow("bin", binImage);

        // 輪郭抽出
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(binImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        // 検出された輪郭線の描画
        for (std::vector<std::vector<cv::Point> >::iterator contour = contours.begin(); contour != contours.end(); contour++) {
            cv::polylines(in_img, *contour, true, cv::Scalar(0, 255, 0), 2);
        }

        // 輪郭が四角形かの判定
        for (std::vector<std::vector<cv::Point> >::iterator contour = contours.begin(); contour != contours.end(); contour++) {
            // 輪郭を直線近似
            std::vector<cv::Point> approx;
            cv::approxPolyDP(cv::Mat(*contour), approx, 50.0, true);
            // 近似が4線かつ面積が一定以上なら四角形
            double area = cv::contourArea(approx);
            if (approx.size() == 4 && area > 1000.0) {
                cv::polylines(in_img, approx, true, cv::Scalar(255, 0, 0), 2);
                std::stringstream sst;
                sst << "area : " << area;
                cv::putText(in_img, sst.str(), approx[0], CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 128, 0));
            }
        }
        cv::imshow("in_img", in_img);
        cv::waitKey(1);
    }

public:
    UmbDetector(ros::NodeHandle nh = ros::NodeHandle()) : it_(nh), node_handle(ros::NodeHandle())
    {
        //img_sub_ = it_.subscribe("/head_camera/rgb/image_raw", 3, &UmbDetector::imageCallBack, this);
        img_sub_ = it_.subscribe("image", 3, &UmbDetector::imageCallBack, this);
        cv::namedWindow("Fast", 1);

        chatter_pub = node_handle.advertise<std_msgs::String>("chatter", 1000);
    }

    int calcDist(const int col[3], int r, int g, int b)
    {
        return (col[0] - r) * (col[0] - r) + (col[1] - g) * (col[1] - g) + (col[2] - b) * (col[2] - b);
    }

    void publishDetectBlue()
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "blue";
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);
    }

private:
    image_transport::Subscriber img_sub_;
    image_transport::ImageTransport it_;
    ros::NodeHandle node_handle;
    ros::Publisher chatter_pub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "umb_detector_node");
    UmbDetector mcp;
    ros::spin();
}
