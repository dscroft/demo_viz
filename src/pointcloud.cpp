#include <algorithm>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"

#include "color_scale.hpp"

class PointCloudViz : public rclcpp::Node
{
public:
    PointCloudViz() : Node("pointcloud_viz")
    {
        // parameters
        scale_ = this->declare_parameter<float>("scale", 1.f);
        int width = this->declare_parameter<int>("width", 640);
        int height = this->declare_parameter<int>("height", 480);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud_viz/cloud", 10, std::bind(&PointCloudViz::topic_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/pointcloud_viz/image", 10);

        image_ = cv::Mat(cv::Size(width, height), CV_8UC3);

        mn_ = std::numeric_limits<float>::max();
        mx_ = std::numeric_limits<float>::min();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    cv::Mat image_;
    float mn_, mx_, scale_;

    void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Pointcloud visualiser getting a message" );

        // Convert PointCloud2 ROS->PCL
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // create blank rgb image
        image_ = cv::Mat::zeros(image_.size(), image_.type());

        //draw lines
        /*for (int i = 0; i < cloud.size(); i++)
        {
            cv::Point2d pt1(cloud[i].x, cloud[i].y);
            cv::Point2d pt2(cloud[(i+1)%cloud.size()].x, cloud[(i+1)%cloud.size()].y);
            cv::line(image_, pt1, pt2, cv::Scalar(255, 255, 255), 1);
        }*/
        cv::line( image_, cv::Point2d(0, image_.rows/2), 
                          cv::Point2d(image_.cols, image_.rows/2), 
                          cv::Scalar(50, 50, 50), 1 );

        cv::line( image_, cv::Point2d( image_.cols/2, 0), 
                          cv::Point2d( image_.cols/2, image_.rows), 
                          cv::Scalar(50, 50, 50), 1 );

        // iterate over all points in the cloud
      
        
        const int cx = image_.cols / 2;
        const int cy = image_.rows / 2;

        float mn = std::numeric_limits<float>::max();
        float mx = std::numeric_limits<float>::min();
        for( const auto& p : cloud )
        {
            if( !std::isfinite(p.z) ) continue;

            // set pixel in image at 10,10 to green
            int x = static_cast<int>(p.x*scale_+0.5) + cx;
            int y = static_cast<int>(p.y*scale_+0.5) + cy;

            if( x < 0 || x >= image_.cols || y < 0 || y >= image_.rows ) continue;

            image_.at<cv::Vec3b>(y, x) = get_color( p.z, mn_, mx_ );

            mn = std::min(mn, p.z);
            mx = std::max(mx, p.z);
        }

        mn_ = mn;
        mx_ = mx;

        image_.at<cv::Vec3b>(cy, cx) = cv::Vec3b(255, 255, 255);

        // flip image vertically
        cv::flip(image_, image_, 0);

        sensor_msgs::msg::Image::SharedPtr out_msg;
        out_msg = cv_bridge::CvImage(msg->header, "bgr8", image_).toImageMsg();
        pub_->publish(*out_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudViz>());
    rclcpp::shutdown();
    return 0;
}