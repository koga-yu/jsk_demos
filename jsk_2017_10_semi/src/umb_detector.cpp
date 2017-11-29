#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"

#include "std_msgs/String.h"
#include "jsk_2017_10_semi/umb_pos.h"
#include <sstream>

class UmbDetector
{
private:
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO("Received image");
        cv::Mat in_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;

        cv::Mat grayImage;
        cv::cvtColor(in_img, grayImage, CV_BGR2GRAY);

        cv::Mat result_img;// = new cv::Mat(in_img.Height - tmp_img.Height + 1, in_img.Width - tmp_img.Width + 1, MatrixType.F32C1);

        cv::matchTemplate(in_img, tmp_img, result_img, CV_TM_SQDIFF);

        cv::Rect roi_rect(0, 0, tmp_img.cols, tmp_img.rows);
        cv::Point max_pt;
        double maxVal;
        cv::minMaxLoc(result_img, NULL, &maxVal, NULL, &max_pt);
        roi_rect.x = max_pt.x;
        roi_rect.y = max_pt.y;
        cv::rectangle(in_img, roi_rect, cv::Scalar(0, 0, 255, 3));

        cv::imshow("in_img", in_img);
        cv::waitKey(1);

        /*
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
            cv::Rect rect = cv::boundingRect(approx);
            cv::rectangle(in_img, cvPoint(rect.x, rect.y),
                cvPoint(rect.x + rect.width, rect.y + rect.height), CV_RGB(255, 0, 0), 2);
            this->detectUmbrella(rect);
        }
        cv::imshow("in_img", in_img);
        cv::waitKey(1);
        */
    }

public:
    UmbDetector(ros::NodeHandle nh = ros::NodeHandle()) : it_(nh), node_handle(ros::NodeHandle())
    {
        //img_sub_ = it_.subscribe("/head_camera/rgb/image_raw", 3, &UmbDetector::imageCallBack, this);
        img_sub_ = it_.subscribe("image", 3, &UmbDetector::imageCallBack, this);
        cv::namedWindow("Fast", 1);

        chatter_pub = node_handle.advertise<jsk_2017_10_semi::umb_pos>("umb_pos", 1000);

        tmp_img = cv::imread("/home/kogatti/semi_ws/src/jsk_demos/jsk_2017_10_semi/picture/umb_handle.png", CV_LOAD_IMAGE_GRAYSCALE);
        if (tmp_img.empty()){
            std::cout << "couldn't read the image. ./../picture/umb_handle.png" << std::endl;
            return;
        }
    }

    int calcDist(const int col[3], int r, int g, int b)
    {
        return (col[0] - r) * (col[0] - r) + (col[1] - g) * (col[1] - g) + (col[2] - b) * (col[2] - b);
    }

    void detectUmbrella(cv::Rect rect) const
    {
        if (rect.width / rect.height > 0.1 and rect.width / rect.height < 0.2) {
            double screen_x = rect.x + rect.width / 2.0;
            double screen_y = rect.y + rect.height * 0.9;

            double ratio = umb_length / rect.height;
            double handle_x = ideal_dist * ideal_height / rect.height;
            double handle_y = screen_x * ratio;
            double handle_z = screen_y * ratio;

            this->publishDetectUmb(handle_x, handle_y, handle_z);
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "umb_detector_node");
    UmbDetector mcp;
    ros::spin();
}
