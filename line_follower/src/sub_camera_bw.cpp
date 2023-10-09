#include <memory>
#include <vector>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "line_follower_interfaces/msg/contour.hpp"
#include "line_follower_interfaces/msg/histogram.hpp"

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class SubscriberBW : public rclcpp::Node
{
  public:
    SubscriberBW() : Node("sub_bw_image"){
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/BWimage", 10, std::bind(&SubscriberBW::img_callback, this, _1));

        publisher_ = this->create_publisher<line_follower_interfaces::msg::Histogram>("/histogram", 10);
        ContourPublisher_ = this->create_publisher<line_follower_interfaces::msg::Contour>("/contourMoment", 10);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<line_follower_interfaces::msg::Histogram>::SharedPtr publisher_;
    rclcpp::Publisher<line_follower_interfaces::msg::Contour>::SharedPtr ContourPublisher_;

    void img_callback(const sensor_msgs::msg::Image::SharedPtr image_msg) const {
        // Convert sensor_msgs/Image to cv::Mat using cv_bridge
        cv_bridge::CvImagePtr cv_image_ptr;
        try {
            // convert sensor_msgs/msg/Image to CvImage
            cv_image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if(cv_image_ptr->image.empty()) {
            RCLCPP_WARN(this->get_logger(), "image empty !");
            return;
        }

        cv::imshow("/BWimage image", cv_image_ptr->image);
        cv::waitKey(1);

        // create_histogram(cv_image_ptr->image);
        computeContours(cv_image_ptr->image);
    }

    void handleContours(const std::vector<std::vector<cv::Point>> contours,const std::vector<cv::Vec4i> hierarchy, const cv::Size image_size) const {
        cv::Mat drawing = cv::Mat::zeros(image_size, CV_8UC3);
        cv::Scalar green_color = cv::Scalar(34, 139, 34);

        cv::drawContours(drawing, contours, 0, green_color, 2, cv::LINE_8, hierarchy, 0);

        computeMoment(contours, drawing);
    }

    void computeMoment(const std::vector<std::vector<cv::Point>> contours, cv::Mat drawing) const {
        // Calculate the moments of the contour
        cv::Moments contourMoments = cv::moments(contours[0]);

        // Calculate the centroid (center of mass) of the contour
        double cx = contourMoments.m10 / contourMoments.m00;
        double cy = contourMoments.m01 / contourMoments.m00;

        // Define the circle's center and radius
        cv::Point circle_center(cx, cy);
        int radius = 15;
        cv::Scalar red_color(0, 0, 255);

        // draw center line
        cv::Size drawing_size = drawing.size();
        cv::Point start_center_line(drawing_size.width/2, 0);
        cv::Point end_center_line(drawing_size.width/2, drawing_size.height);

        cv::circle(drawing, circle_center, radius, red_color, -1); // -1 fills the circle

        cv::line(drawing, start_center_line, end_center_line, red_color, 2);

        // draw line from bottom center to centroid
        cv::line(drawing, circle_center, end_center_line, red_color, 2);

        cv::imshow("Contours", drawing);

        publishContourData(circle_center, drawing.size());
    }

    void publishContourData(const cv::Point circle_center, const cv::Size image) const {
        // prpare and publish contour message
        auto contour_message_published = std::make_shared<line_follower_interfaces::msg::Contour>();
        contour_message_published->centroid_x = circle_center.x;
        contour_message_published->centroid_y = circle_center.y;
        contour_message_published->image_width =  image.width;
        contour_message_published->image_height = image.height;

        ContourPublisher_->publish(*contour_message_published);

        RCLCPP_INFO(this->get_logger(), "Contour data published !");
    }

    void computeContours(const cv::Mat image) const {
        RCLCPP_INFO(this->get_logger(), "In compute edge method !");
    
        bool recoveryMode = false;

        // convert image to CV_8UC1
        cv::Mat binaryImage;
        if (image.type() != CV_8UC1) {
            cv::cvtColor(image, binaryImage, cv::COLOR_BGR2GRAY);
        }

        // find contours of the objects in the image
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        try {
            cv::findContours(binaryImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            if(contours.size() > 0) {
                if(recoveryMode){ // reset recoveryMode
                    recoveryMode = false;
                }
                // handle the contours moments / centroid computation / drawing
                handleContours(contours, hierarchy, image.size());
            } else {
                // goes into recovery Mode
                recoveryMode = true;
            }
        }catch(std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] findcountours method failed ! (%s)", e.what());
        }
    }

    // old method
    void create_histogram(const cv::Mat image ) const {
        cv::Size s = image.size();
        RCLCPP_INFO(this->get_logger(), "In Create histogram method: w %d h %d", s.width, s.height);

        int value;
        int cols = image.cols;
        int rows = image.rows;

        std::vector<int> histogram(cols, 0); // init a vector of size width with zero values

        for (int col=0; col < cols; col++) {
            for(int row=0; row < rows; row++){
                value = image.at<int>(row, col);
                if(value != 0) {
                    histogram[col] += 1;
                }
            }
        }
        
        // prpare and publish histogram message
        auto hist_message_published = std::make_shared<line_follower_interfaces::msg::Histogram>();
        hist_message_published->data = histogram;
        hist_message_published->length = histogram.size();

        publisher_->publish(*hist_message_published);

        RCLCPP_INFO(this->get_logger(), "Histogram data published !");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberBW>());
  rclcpp::shutdown();
  return 0;
}