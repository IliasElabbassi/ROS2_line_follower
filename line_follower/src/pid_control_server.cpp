
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "line_follower_interfaces/srv/angle.hpp"

class PID_controler : public rclcpp::Node
{
public:
    PID_controler() : Node("pid_controller") {
        RCLCPP_INFO(this->get_logger(), "Launched PID controler Node !");
        PIDservice = this->create_service<line_follower_interfaces::srv::Angle>("/getPidOutput",  std::bind(&PID_controler::calculateControlOutput, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Service<line_follower_interfaces::srv::Angle>::SharedPtr PIDservice;
    
    double kp_ = 0.5;
    double ki_ = 0;
    double kd_ = 0;
    double setpoint_ = 0.0;

    double integral_ = 0.0;
    double previous_error_ = 0.0;

    void calculateControlOutput(const std::shared_ptr<line_follower_interfaces::srv::Angle::Request> request, 
                                        std::shared_ptr<line_follower_interfaces::srv::Angle::Response> response) {
        integral_ += request->angle;
        double derivative = request->angle - previous_error_;
        double control_output = (kp_ * request->angle) + (ki_ * integral_) + (kd_ * derivative);
        previous_error_ = request->angle;

        response->pid_output = control_output;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PID_controler>());
  rclcpp::shutdown();
  return 0;
}
