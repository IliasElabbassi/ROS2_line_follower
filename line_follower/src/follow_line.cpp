#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "line_follower_interfaces/msg/contour.hpp"

#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class FollowLine : public rclcpp::Node
{
    public:
        FollowLine() : Node("follow_line") {
            RCLCPP_INFO(this->get_logger(), "Launched line following !");

            CmdVelpublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            ContourSubscription = this->create_subscription<line_follower_interfaces::msg::Contour>("/contourMoment", 10, std::bind(&FollowLine::handleMovement, this, _1));
        }

    private:
        rclcpp::Subscription<line_follower_interfaces::msg::Contour>::SharedPtr ContourSubscription;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr CmdVelpublisher_;

        void handleMovement(const line_follower_interfaces::msg::Contour::SharedPtr contour_moment_msg) const{
            // RCLCPP_INFO(this->get_logger(), "Movement Handling : centroid (%d, %d) !", contour_moment_msg->centroid_x, contour_moment_msg->centroid_y);

            // {hypotenuse, adjacent, opposée}
            std::vector<int> vector_distance = compute_distances(contour_moment_msg);

            int side = check_lef_right(contour_moment_msg); // 1 : left   2 : right

            if (side == 1) {
                std::cout << "side : left" << std::endl;
            } else {
                std::cout << "side : right" << std::endl;
            }

            // for(auto val : vector_distance) {
            //     std::cout << val << " ";
            // }
            // std::cout << std::endl;
            
            int opposee = vector_distance[2];
            int hypotenuse = vector_distance[0];

            // std::cout << "o : " << opposee << "h : " << hypotenuse << std::endl;

            double angle = compute_angle_degree(opposee, hypotenuse);

            std::cout << "angle CBD : " << angle << std::endl;
        }

        std::vector<int> compute_distances(const line_follower_interfaces::msg::Contour::SharedPtr contour_moment_msg) const {
            cv::Point A, B, C, D;
            A.x = contour_moment_msg->image_width / 2;
            A.y = 0;

            // std::cout << "A : (" << A.x << "," << A.y << ")" << std::endl;

            B.x = contour_moment_msg->image_width / 2;
            B.y = contour_moment_msg->image_height;

            // std::cout << "B : (" << B.x << "," << B.y << ")" << std::endl;

            C.x = contour_moment_msg->centroid_x;
            C.y = contour_moment_msg->centroid_y;

            // std::cout << "C : (" << C.x << "," << C.y << ")" << std::endl;

            D.x = B.x;
            D.y = C.y;

            // std::cout << "D : (" << D.x << "," << D.y << ")" << std::endl;

            // hypotenuse
            int distance_BC = compute_length(B, C);
            // std::cout << "BC : " << distance_BC << std::endl;

            // adjacent
            int distance_BD = compute_length(B, D);
            // std::cout << "BD : " << distance_BD << std::endl;

            /// opposée
            int distance_CD = compute_length(C, D);
            // std::cout << "CD : " << distance_CD << std::endl;

            std::vector<int> distance_vector;

            distance_vector.push_back(distance_BC);
            distance_vector.push_back(distance_BD);
            distance_vector.push_back(distance_CD);

            return distance_vector;
        }

        int compute_length(const cv::Point point1, const cv::Point point2) const{
            return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2));
        }

        double compute_angle_degree(const int distance1, const int distance2) const {
            double param = (double) distance1 / distance2;

            // std::cout << "in compute angle : " << distance1 << " / " << distance2 << "= " << param << std::endl;

            return (std::asin(param)*180/3.1415);
        }

        int check_lef_right(const line_follower_interfaces::msg::Contour::SharedPtr contour_moment_msg) const {
            cv::Point C;

            C.x = contour_moment_msg->centroid_x;
            C.y = contour_moment_msg->centroid_y;

            int width_center = contour_moment_msg->image_width / 2;

            if(C.x >= width_center) {
                return 2; // centroid in the right of the camera
            }

            return 1; // centroid in the left of the camera
        }

        void move(){
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.linear.x = 0.2; // Move forward at 0.5 m/s
            twist_msg.angular.z = 0.2; // Rotate at 0.1 rad/s

            CmdVelpublisher_->publish(twist_msg);

            // RCLCPP_INFO(this->get_logger(), "Publishing cmd vel");
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowLine>());
  rclcpp::shutdown();
  return 0;
}
