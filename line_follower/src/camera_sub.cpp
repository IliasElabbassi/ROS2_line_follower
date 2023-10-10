#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class CameraSubscriber : public rclcpp::Node
{
  public:

    CameraSubscriber()
    : Node("camera_subscriber")
    {
      RCLCPP_INFO(this->get_logger(), "Launched Image processing !");

      subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10,
                                          std::bind(&CameraSubscriber::image_callback, this, _1));

      BWpublisher_ = this->create_publisher<sensor_msgs::msg::Image>("BWimage", 10);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr BWpublisher_;
    // mutable cv::Mat binary_image; // mutable so it can be acessed within a const method

    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg) const {
        // Convert sensor_msgs/Image to cv::Mat using cv_bridge
        cv_bridge::CvImagePtr cv_image_ptr;
        try {
            // convert sensor_msgs/msg/Image to CvImage
            cv_image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // CvImagePtr->image is cv:Mat
        displayImage(cv_image_ptr->image);

        // RCLCPP_INFO(this->get_logger(), "Image height & width ('%d', '%d')", image_msg->height, image_msg->width);
    }

    int displayImage(cv::Mat image) const {
      if(image.empty()) {
        RCLCPP_WARN(this->get_logger(), "image empty !");
        return -1;
      }

      int blur_kernel_size = 9;
      int threshold_value = 180; // change this value to better fit your camera / world / lighting
      cv::Mat blurred_image, gray_scale_image, binary_image;


      // blur orignal image
      cv::GaussianBlur(image, blurred_image, cv::Size(blur_kernel_size, blur_kernel_size), 0);
      // turn blurred image into gray scale image
      cv::cvtColor(blurred_image, gray_scale_image, cv::COLOR_BGR2GRAY);
      // turn gray scale image into B&W image
      cv::threshold(gray_scale_image, binary_image, threshold_value, 255, cv::THRESH_BINARY);

      // // display the cv::Mat with cv::imshow
      cv::imshow("original image", image);
      // cv::imshow("Blurred Image", blurred_image);
      // cv::imshow("Gray scale Image", gray_scale_image);
      cv::imshow("B&W Image", binary_image);
  
      // convert opencv to sensor Image type for publishing
      sensor_msgs::msg::Image::SharedPtr bw_image_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", binary_image).toImageMsg();
    
      // publish B&W image to publisher
      BWpublisher_->publish(*bw_image_ptr);
      
      // give some time to the image to display and update properly
      cv::waitKey(1);

      return 0;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}