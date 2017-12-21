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
#include <cmath>

typedef struct UmbCandidate {
    double center_x;
    double center_y;
    double size;
    double coeff;
} UmbCandidate;

class UmbDetector
{
private:
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO("Received image");
        cv::Mat in_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;

        int tmp_num = 0;
        UmbCandidate tmp_umbcandidate;
        tmp_umbcandidate.coeff = 0;
        UmbCandidate tmp_umbcandidate_up;

        for (int i = 0; i < 5; i++) {
            tmp_umbcandidate_up
                = this->calcUmbCandidate(in_img, 1.0 - static_cast<double>(i) * 0.1);
            if (tmp_umbcandidate_up.coeff > tmp_umbcandidate.coeff) {
                tmp_umbcandidate = tmp_umbcandidate_up;
            }
        }
        double tmp_max_size = tmp_umbcandidate.size;
        for (int i = 0; i < 10; i++) {
            tmp_umbcandidate_up
                = this->calcUmbCandidate(in_img, tmp_max_size - 0.1 + 0.01 * static_cast<double>(i));
            if (tmp_umbcandidate_up.coeff > tmp_umbcandidate.coeff) {
                tmp_umbcandidate = tmp_umbcandidate_up;
            }
        }
        tmp_max_size = tmp_umbcandidate.size;
        for (int i = 0; i < 10; i++) {
            tmp_umbcandidate_up
                = this->calcUmbCandidate(in_img, tmp_max_size - 0.01 + 0.001 * static_cast<double>(i));
            if (tmp_umbcandidate_up.coeff > tmp_umbcandidate.coeff) {
                tmp_umbcandidate = tmp_umbcandidate_up;
            }
        }
        this->drawDetectedUmb(in_img, tmp_umbcandidate);
        if (tmp_umbcandidate.coeff > this->match_coeff_thresh) {
            double real_dist = this->ideal_dist / tmp_umbcandidate.size;
            real_dist = std::sqrt(real_dist * real_dist - camera_hand_height_diff * camera_hand_height_diff);
            double angle = -(tmp_umbcandidate.center_x - in_img.cols / 2.0) * ideal_angle_ratio / tmp_umbcandidate.size;
            this->publishDetectUmb(real_dist * std::cos(angle), real_dist * 0.9 * std::sin(angle),
                ideal_height);
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

        chatter_pub = node_handle.advertise<jsk_2017_10_semi::umb_pos>("detected_umb_pos", 1000);

        tmp_img = cv::imread("/home/kogatti/semi_ws/src/jsk_demos/jsk_2017_10_semi/picture/umb_handle.png", 1);
        if (tmp_img.empty()) {
            std::cout << "couldn't read the image. ./../picture/umb_handle.png" << std::endl;
            return;
        }
        cv::imshow("tmp_img", tmp_img);
    }

    void publishDetectUmb(float x, float y, float z) const
    {
        jsk_2017_10_semi::umb_pos msg;
        msg.x = x;
        msg.y = y;
        msg.z = z;

        chatter_pub.publish(msg);
    }

    UmbCandidate calcUmbCandidate(cv::Mat& in_img, double size)
    {
        cv::Mat tmp_img_resized;
        cv::resize(tmp_img, tmp_img_resized, cv::Size(), size, size);
        cv::matchTemplate(in_img, tmp_img_resized, result_img, CV_TM_CCOEFF_NORMED);

        cv::Point max_pt;
        double max_val;
        cv::minMaxLoc(result_img, NULL, &max_val, NULL, &max_pt);

        return UmbCandidate{max_pt.x + tmp_img_resized.cols * size / 2.0,
            max_pt.y + tmp_img_resized.rows * size / 2.0,
            size, max_val};
    }

    void drawDetectedUmb(cv::Mat& in_img, UmbCandidate& umb_candidate)
    {
        double size = umb_candidate.size;
        cv::Rect roi_rect(0, 0, tmp_img.cols * size, tmp_img.rows * size);
        cv::Point max_pt(umb_candidate.center_x - tmp_img.cols * size / 2.0,
            umb_candidate.center_y - tmp_img.rows * size / 2.0);
        roi_rect.x = umb_candidate.center_x - tmp_img.cols * size / 2.0;
        roi_rect.y = umb_candidate.center_y - tmp_img.rows * size / 2.0;

        if (umb_candidate.coeff > this->match_coeff_thresh) {
            cv::rectangle(in_img, roi_rect, cv::Scalar(0, 0, 255, 3));
        } else {
            cv::rectangle(in_img, roi_rect, cv::Scalar(255, 0, 0, 3));
        }

        std::stringstream stream;
        stream << umb_candidate.size << " " << umb_candidate.coeff;

        cv::putText(in_img, stream.str(), max_pt, 1, 3.0, cv::Scalar(100, 255, 255, 3));
    }

private:
    image_transport::Subscriber img_sub_;
    image_transport::ImageTransport it_;
    ros::NodeHandle node_handle;
    ros::Publisher chatter_pub;

    cv::Mat result_img;
    cv::Mat tmp_img;

    double umb_length = 800.0;
    double ideal_height = 700.0;
    double ideal_dist = 750.0;
    double ideal_angle_ratio = std::atan2(120.0, ideal_dist) / 95.0;
    double match_coeff_thresh = 0.65;
    double search_size_width = 0.02;
    double camera_hand_height_diff = 300.0;
    int search_cnt_max = 10;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "umb_detector_node");
    UmbDetector mcp;
    ros::spin();
}
