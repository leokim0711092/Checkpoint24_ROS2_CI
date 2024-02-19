#include<geometry_msgs/msg/twist.hpp>
#include<geometry_msgs/msg/point.hpp>
#include<tortoisebot_waypoints/action/waypoint_action.hpp>
#include<std_msgs/msg/empty.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include "rclcpp/executors.hpp"
#include "rclcpp/rate.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <cmath>

using namespace std::placeholders;


class WaypointActionClass : public rclcpp::Node
{
public:
    using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
    using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<WaypointAction>;

    WaypointActionClass(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()):
    Node("Waypoint", options)
    {

        this->_as = rclcpp_action::create_server<WaypointAction>(
        this,
        "tortoisebot_as", 
        std::bind(&WaypointActionClass::goal_response, this, _1, _2),
        std::bind(&WaypointActionClass::goal_cancel, this, _1),
        std::bind(&WaypointActionClass::handle_accepted, this, _1)
        );

        _pub_cmd_vel =this-> create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        _sub_odom = this-> create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&WaypointActionClass::clbk_odom, this, std::placeholders::_1));
        _yaw_precision = M_PI / 90; 
        _dist_precision = 0.05;
    }

private:
    rclcpp_action::Server<WaypointAction>::SharedPtr _as;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_cmd_vel;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odom;
    

    // robot state variables
    geometry_msgs::msg::Point _position;
    float _yaw;

    // machine state
    std::string _state = "idle";

    // goal
    geometry_msgs::msg::Point _des_pos;

    // parameters
    float _yaw_precision;
    float _dist_precision;

    void clbk_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // position
        _position = msg->pose.pose.position;

        // yaw
        tf2::Quaternion tf_quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, tf_quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
        _yaw = yaw;
    }

    rclcpp_action::GoalResponse goal_response(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const WaypointAction::Goal> goal){
        RCLCPP_INFO(this->get_logger(), "Received goal request with secs");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
    }

    rclcpp_action::CancelResponse goal_cancel(const std::shared_ptr<GoalHandleWaypoint> go_cancel){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)go_cancel;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle){

        std::thread{std::bind(&WaypointActionClass::execute, this, _1), goal_handle}.detach();
    
    }

    void execute( const std::shared_ptr<GoalHandleWaypoint> goal_handle)
    {
        rclcpp::Rate _rate(25);
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<WaypointAction::Feedback>();
        auto result = std::make_shared<WaypointAction::Result>();
        // helper variables
        bool success = true;

        // define desired position and errors
        _des_pos = goal->position;
        float desired_yaw = atan2(_des_pos.y - _position.y, _des_pos.x - _position.x);
        float err_pos = sqrt(pow(_des_pos.y - _position.y, 2) + pow(_des_pos.x - _position.x, 2));
        float err_yaw = desired_yaw - _yaw;

        // perform task
        while (err_pos > _dist_precision && success)
        {
            // update vars
            desired_yaw = atan2(_des_pos.y - _position.y, _des_pos.x - _position.x);
            err_yaw = desired_yaw - _yaw;
            err_pos = sqrt(pow(_des_pos.y - _position.y, 2) + pow(_des_pos.x - _position.x, 2));
            RCLCPP_INFO(get_logger(), "Current Yaw: %f", _yaw);
            RCLCPP_INFO(get_logger(), "Desired Yaw: %f", desired_yaw);
            RCLCPP_INFO(get_logger(), "Error Yaw: %f", err_yaw);
            // logic goes here
            if (goal_handle->is_canceling())
            {
                // cancelled
                RCLCPP_INFO(get_logger(), "The goal has been cancelled/preempted");
                goal_handle->canceled(result);
                success = false;
            }
            else if (fabs(err_yaw) > _yaw_precision)
            {
                // fix yaw
                RCLCPP_INFO(get_logger(), "fix yaw");
                _state = "fix yaw";
                auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
                twist_msg->angular.z = 0.65 * (err_yaw > 0 ? 1 : -1);
                _pub_cmd_vel->publish(std::move(twist_msg));
            }
            else
            {
                // go to point
                RCLCPP_INFO(get_logger(), "go to point");
                _state = "go to point";
                auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
                twist_msg->linear.x = 1.0;
                twist_msg->angular.z = 0;
                _pub_cmd_vel->publish(std::move(twist_msg));
            }

            feedback->state = _state;
            feedback->position = _position;
            feedback->yaw = _yaw;
            // send feedback
            goal_handle->publish_feedback(feedback);

            // loop rate
            _rate.sleep();
        }

        // stop
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 0;
        twist_msg->angular.z = 0;
        _pub_cmd_vel->publish(std::move(twist_msg));

        // return success
        if (success)
        {
            result->success = true;
            goal_handle->succeed(result);
        }
    }
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointActionClass>());
    rclcpp::shutdown();
    return 0;

}



