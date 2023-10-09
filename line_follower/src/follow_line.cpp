#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "line_follower_interfaces/msg/contour.hpp"

using std::placeholders::_1;

class FollowLine : public rclcpp::Node
{
    public:
        FollowLine() : Node("follow_line") {
            CmdVelpublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            ContourSubscription = this->create_subscription<line_follower_interfaces::msg::Contour>("/contourMoment", 10, std::bind(&FollowLine::handleMovement, this, _1));
        }

    private:
        rclcpp::Subscription<line_follower_interfaces::msg::Contour>::SharedPtr ContourSubscription;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr CmdVelpublisher_;

        void handleMovement(const line_follower_interfaces::msg::Contour::SharedPtr contour_moment_msg) const{
            RCLCPP_INFO(this->get_logger(), "Movement Handling : centroid (%d, %d) !", contour_moment_msg->centroid_x, contour_moment_msg->centroid_y);

        }


        void move(){
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.linear.x = 0.2; // Move forward at 0.5 m/s
            twist_msg.angular.z = 0.2; // Rotate at 0.1 rad/s

            CmdVelpublisher_->publish(twist_msg);

            RCLCPP_INFO(this->get_logger(), "Publishing cmd vel");
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowLine>());
  rclcpp::shutdown();
  return 0;
}
