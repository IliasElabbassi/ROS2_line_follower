#include <memory>
#include <cstdlib>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "line_follower_interfaces/msg/contour.hpp"
#include "line_follower_interfaces/srv/angle.hpp"

#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class FollowLine : public rclcpp::Node
{
    public:
        FollowLine() : Node("follow_line") {
            RCLCPP_INFO(this->get_logger(), "Launched line following !");


            callback_group_contour_subscription = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_service_client = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            auto contour_sub_opt = rclcpp::SubscriptionOptions();
            contour_sub_opt.callback_group = callback_group_contour_subscription;

            auto service_client_opt = rclcpp::SubscriptionOptions();
            service_client_opt.callback_group = callback_group_service_client;

            CmdVelpublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
           
            ContourSubscription = this->create_subscription<line_follower_interfaces::msg::Contour>("/contourMoment", rclcpp::QoS(10), std::bind(&FollowLine::handleMovement, this, _1), contour_sub_opt);
            
            PID_client = this->create_client<line_follower_interfaces::srv::Angle>("/getPidOutput", rmw_qos_profile_services_default,  callback_group_service_client);
        }

    private:
        rclcpp::Subscription<line_follower_interfaces::msg::Contour>::SharedPtr ContourSubscription;
        rclcpp::Client<line_follower_interfaces::srv::Angle>::SharedPtr PID_client;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr CmdVelpublisher_;
        rclcpp::CallbackGroup::SharedPtr callback_group_contour_subscription;
        rclcpp::CallbackGroup::SharedPtr callback_group_service_client;
        double computed_angle = 0;
        int computed_side = 1;
        
        void handleMovement(const line_follower_interfaces::msg::Contour::SharedPtr contour_moment_msg) {
            // {hypotenuse, adjacent, opposée}
            std::vector<int> vector_distance = compute_distances(contour_moment_msg);

            computed_side = check_lef_right(contour_moment_msg); // 1 : left   2 : right

            if (computed_side == 1) {
                std::cout << "side : left" << std::endl;
            } else {
                std::cout << "side : right" << std::endl;
            }

            int opposee = vector_distance[2];
            int hypotenuse = vector_distance[0];

            computed_angle = compute_angle_degree(opposee, hypotenuse);

            pid_request();

            std::cout << "angle CBD : " << computed_angle << std::endl;
        }

        std::vector<int> compute_distances(const line_follower_interfaces::msg::Contour::SharedPtr contour_moment_msg) const {
            cv::Point A, B, C, D;
            A.x = contour_moment_msg->image_width / 2;
            A.y = 0;

            B.x = contour_moment_msg->image_width / 2;
            B.y = contour_moment_msg->image_height;

            C.x = contour_moment_msg->centroid_x;
            C.y = contour_moment_msg->centroid_y;

            D.x = B.x;
            D.y = C.y;

            // hypotenuse
            int distance_BC = compute_length(B, C);
            // adjacent
            int distance_BD = compute_length(B, D);
            /// opposée
            int distance_CD = compute_length(C, D);

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

        void pid_request() {
            std::cout << "in pid_request" << std::endl;

            auto request = std::make_shared<line_follower_interfaces::srv::Angle::Request>();
            request->angle = computed_angle;
            
            std::chrono::seconds one_second(1);

            if(!PID_client->wait_for_service(one_second)){
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "PID service not available, waiting again...");
            }

            // Make a service request
            auto future_result = PID_client->async_send_request(request);
 
            // Process the response
            // auto node_base = this->get_node_base_interface();

            std::chrono::seconds three_second(3);

            std::future_status status = future_result.wait_for(three_second);

            if (status == std::future_status::ready) {
                RCLCPP_INFO(this->get_logger(), "Received response");
                RCLCPP_INFO(this->get_logger(), "pid_output : %f", future_result.get()->pid_output);
            }

            // if (rclcpp::spin_until_future_complete(node_base, future_result) == rclcpp::FutureReturnCode::SUCCESS) {
            //     RCLCPP_INFO(this->get_logger(), "pid_output : %f", future_result.get()->pid_output);
            // }

            std::cout << "end pid_request" << std::endl;
        }

        void move(){
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.linear.x = 0.2; // Move forward at 0.5 m/s
            twist_msg.angular.z = 0.2; // Rotate at 0.1 rad/s

            CmdVelpublisher_->publish(twist_msg);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // we must use the MultiThreadedExecutor multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;
    auto follow_line_node = std::make_shared<FollowLine>();
    executor.add_node(follow_line_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
