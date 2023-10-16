#include "rclcpp/rclcpp.hpp"

class PID_controler : public rclcpp::Node
{
public:
    PID_controler() : Node("pid_controller") {
        RCLPP_INFO(this->ger_logger(), "Launched PID controler Node !");
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double setpoint_;

    double integral_ = 0.0;
    double previous_error_ = 0.0;

    double calculateControlOutput(double error) {
        integral_ += error;
        double derivative = error - previous_error_;
        double control_output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        previous_error_ = error;
        return control_output;
    }
}