#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "gtest/gtest.h"
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tortoisebot_waypoints/action/waypoint_action.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class WaypointTest : public ::testing::Test {

public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleWaypoint = rclcpp_action::ClientGoalHandle<WaypointAction>;

  WaypointTest() {

    node = rclcpp::Node::make_shared("test_client");

    this->client_ptr_ =
        rclcpp_action::create_client<WaypointAction>(node, "/tortoisebot_as");

    _sub_odom = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1,
        std::bind(&WaypointTest::odom_callback, this, std::placeholders::_1));

    dest_x = 0.5;
    dest_y = 0.5;
    dest_yaw = atan2(dest_y - cur_pos_y, dest_x - cur_pos_x);
    send_goal();
  }

  bool position_test() {
    while (active) {
      rclcpp::spin_some(node);
    }
    RCLCPP_INFO(node->get_logger(), "The goal x is %f", this->dest_x);
    RCLCPP_INFO(node->get_logger(), "The goal x is %f", this->dest_y);

    float x_error = abs(this->dest_x - this->cur_pos_x);
    float y_error = abs(this->dest_y - this->cur_pos_y);
    float yaw_error = abs(this->dest_yaw - this->cur_yaw);

    while(x_error == this->dest_x || y_error == this->dest_y ||  yaw_error == this->dest_yaw){
        RCLCPP_INFO(node->get_logger(), "X_error is %f", x_error);
        RCLCPP_INFO(node->get_logger(), "Y_error is %f", y_error);
        RCLCPP_INFO(node->get_logger(), "Yaw_error is %f", yaw_error);
    }


    if (x_error <= 0.4 && y_error <= 0.4 && yaw_error <= 0.4 ) {
      return true;
    }

    return false;
  }

protected:
  std::shared_ptr<rclcpp::Node> node;

private:
  bool active;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odom;
  rclcpp_action::Client<WaypointAction>::SharedPtr client_ptr_;
  float cur_pos_x = 0.0;
  float cur_pos_y = 0.0;
  float cur_yaw = 0.0;
  float dest_x = 0.0;
  float dest_y = 0.0;
  float dest_yaw = 0.0;
  geometry_msgs::msg::Point position;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    active = true;
    // position
    this->position = msg->pose.pose.position;
    this->cur_pos_x = this->position.x;
    this->cur_pos_y = this->position.y;
    RCLCPP_INFO(node->get_logger(), "Odom active");

    // yaw
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, tf_quaternion);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
    this->cur_yaw = yaw;
  }

  void send_goal() {
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_INFO(node->get_logger(), "waiting action server");
    }

    auto goal_msg = WaypointAction::Goal();
    goal_msg.position.x = this->dest_x;
    goal_msg.position.y = this->dest_y;

    RCLCPP_INFO(node->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<WaypointAction>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&WaypointTest::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&WaypointTest::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&WaypointTest::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(
      const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleWaypoint::SharedPtr,
      const std::shared_ptr<const WaypointAction::Feedback> feedback) {
        cur_pos_x = feedback->position.x;
        cur_pos_y = feedback->position.y;
        cur_yaw = feedback->yaw;

      
      }

  void result_callback(const GoalHandleWaypoint::WrappedResult &result) {
    active = false;
  }
};

TEST_F(WaypointTest, position_gtest) { EXPECT_TRUE(position_test()); }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}