#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "std_msgs/String.h"
#include <sstream>

class UmbDetector
{
private:
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO("Received image");
        cv::Mat in_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;

        cv::Mat fast_img = in_img.clone();
        //int threshold = 10;
        //bool nonmax = true;
        //std::vector<cv::KeyPoint> keypoints;
        //cv::FAST(in_img, keypoints, threshold, nonmax);
        //std::vector<cv::KeyPoint>::iterator it_kp = keypoints.begin();
        //for(; it_kp != keypoints.end(); ++it_kp) {
        //    cv::circle(fast_img, cv::Point(it_kp->pt.x, it_kp->pt.y), 1,
        //        cv::Scalar(50, 0, 255), -1);
        //    cv::circle(fast_img, cv::Point(it_kp->pt.x, it_kp->pt.y), 8,
        //        cv::Scalar(50, 0, 255));
        //}
        const int col_r[3] = {255, 0, 0};
        const int col_g[3] = {0, 255, 0};
        const int col_b[3] = {0, 0, 255};
        const int diff_threshold_r = 40000;
        const int diff_threshold_g = 40000;
        const int diff_threshold_b = 40000;

        for (int y = 0; y < fast_img.rows; ++y) {
            for (int x = 0; x < fast_img.cols; ++x) {
                //0B 1G 2R
                int tmp_r = fast_img.data[y * fast_img.step + x * fast_img.elemSize() + 2];
                int tmp_g = fast_img.data[y * fast_img.step + x * fast_img.elemSize() + 1];
                int tmp_b = fast_img.data[y * fast_img.step + x * fast_img.elemSize() + 0];

                bool is_found = false;
                if (this->calcDist(col_g, tmp_r, tmp_g, tmp_b) < diff_threshold_g) {
                    is_found = true;
                }
                if (this->calcDist(col_r, tmp_r, tmp_g, tmp_b) < diff_threshold_r) {
                    is_found = true;
                }
                if (this->calcDist(col_b, tmp_r, tmp_g, tmp_b) < diff_threshold_b) {
                    this->publishDetectBlue();
                    is_found = true;
                }
                if (not is_found) {
                    fast_img.data[y * fast_img.step + x * fast_img.elemSize() + 0] = 0;
                    fast_img.data[y * fast_img.step + x * fast_img.elemSize() + 1] = 0;
                    fast_img.data[y * fast_img.step + x * fast_img.elemSize() + 2] = 0;
                }
                // 画像のチャネル数分だけループ。白黒の場合は1回、カラーの場合は3回　　　　　
                //for(int c = 0; c < fast_img.channels(); ++c){
                //	cout << src.data[ y * fast_img.step + x * fast_img.elemSize() + c ] << endl;
                //}
            }
        }
        cv::imshow("Fast", fast_img);
        cv::waitKey(1);
    }

public:
    UmbDetector(ros::NodeHandle nh = ros::NodeHandle()) : it_(nh), node_handle(ros::NodeHandle())
    {
        img_sub_ = it_.subscribe("/head_camera/rgb/image_raw", 3, &UmbDetector::imageCallBack, this);
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
	std::cout << "before ros::inti" << std::endl;
    ros::init(argc, argv, "umb_detector_node");
	std::cout << "before initialize umbdetector" << std::endl;
    UmbDetector mcp;
    ros::spin();
}
