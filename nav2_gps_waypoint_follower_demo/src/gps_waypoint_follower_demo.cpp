// Copyright (c) 2020 Fetullah Atas
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>
#include "nav2_gps_waypoint_follower_demo/gps_waypoint_follower_demo.hpp"

namespace nav2_gps_waypoint_follower_demo
{
GPSWayPointFollowerClient::GPSWayPointFollowerClient()
: Node("GPSWaypointFollowerClient"), goal_done_(false)
{
  gps_waypoint_follower_action_client_ =
    rclcpp_action::create_client<ClientT>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "follow_gps_waypoints");

  // number of poses that robot will go throug, specified in yaml file
  this->declare_parameter("waypoints", std::vector<std::string>({"0"}));
  this->declare_parameter("autostart",false);
  auto autostart = this->get_parameter("autostart").as_bool();

  waypoints_service = create_service<nav2_msgs::srv::SendGps>(
    "waypoints_send",std::bind(&GPSWayPointFollowerClient::WaypointsCallback,this,
    std::placeholders::_1,std::placeholders::_2));
  
  if (autostart){
    gps_poses_from_yaml_ = loadGPSWaypointsFromYAML();
    RCLCPP_INFO(this->get_logger(),"Created an Instance of GPSWayPointFollowerClient");

    RCLCPP_INFO(this->get_logger(),
    "Loaded %i GPS waypoints from YAML, gonna pass them to FollowGPSWaypoints...",    
    static_cast<int>(gps_poses_from_yaml_.size()));

    startWaypointFollowing(gps_poses_from_yaml_);
  }
}

GPSWayPointFollowerClient::~GPSWayPointFollowerClient()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Destroyed an Instance of GPSWayPointFollowerClient");
}

void GPSWayPointFollowerClient::WaypointsCallback(const std::shared_ptr<nav2_msgs::srv::SendGps::Request> request,
  std::shared_ptr<nav2_msgs::srv::SendGps::Response> response){

    RCLCPP_INFO(get_logger(),"Waypoint GPS service called. Number of waypoints: %i", static_cast<int>(request->gps_poses.size()));
    startWaypointFollowing(request->gps_poses);
    response->result= true;
    return;
  }

void GPSWayPointFollowerClient::startWaypointFollowing(std::vector<geographic_msgs::msg::GeoPose> gps_poses)
{

  if (gps_poses.size()==0){
    return;
  }

  using namespace std::placeholders;

  this->goal_done_ = false;

  if (!this->gps_waypoint_follower_action_client_) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  auto is_action_server_ready =
    gps_waypoint_follower_action_client_->wait_for_action_server(
    std::chrono::seconds(1));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      this->get_logger(), "FollowGPSWaypoints action server is not available."
      " Make sure an instance of GPSWaypointFollower is up and running");
    this->goal_done_ = true;
    return;
  }
  gps_waypoint_follower_goal_ = ClientT::Goal();
  // Send the goal poses
  gps_waypoint_follower_goal_.gps_poses = gps_poses;

  RCLCPP_INFO(
    this->get_logger(),
    "Sending a path of %zu gps_poses:", gps_waypoint_follower_goal_.gps_poses.size());
  for (auto pose : gps_waypoint_follower_goal_.gps_poses) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "\t(%lf, %lf)", pose.position.latitude, pose.position.longitude);
  }

  auto goal_options =
    rclcpp_action::Client<ClientT>::SendGoalOptions();

  goal_options.goal_response_callback = std::bind(
    &GPSWayPointFollowerClient::goalResponseCallback, this, _1);

  goal_options.feedback_callback =
    std::bind(&GPSWayPointFollowerClient::feedbackCallback, this, _1, _2);

  goal_options.result_callback = std::bind(
    &GPSWayPointFollowerClient::resultCallback, this, _1);

  auto future_goal_handle = gps_waypoint_follower_action_client_->async_send_goal(
    gps_waypoint_follower_goal_, goal_options);

}

std::vector<geographic_msgs::msg::GeoPose>
GPSWayPointFollowerClient::loadGPSWaypointsFromYAML()
{
  std::vector<std::string> waypoints_vector =
    this->get_parameter("waypoints").as_string_array();
  std::vector<geographic_msgs::msg::GeoPose> gps_waypoint_msg_vector;

  if (waypoints_vector.size()==0){
    RCLCPP_WARN(get_logger(),"Zero waypoints registered from the configuration file");
    return gps_waypoint_msg_vector;
  }

  for (auto && curr_waypoint : waypoints_vector) {
    //std::cout << curr_waypoint << std::endl;
    try {
      this->declare_parameter(curr_waypoint, std::vector<double>({0}));
      std::vector<double> gps_waypoint_vector =
        this->get_parameter(curr_waypoint).as_double_array();
      // throw exception if incorrect format was detected from yaml file reading
      if (gps_waypoint_vector.size() != 4) {
        std::cout << gps_waypoint_vector.size() << std::endl;
        RCLCPP_FATAL(
          this->get_logger(),
          "GPS waypoint that was loaded from YAML file seems to have incorrect"
          " form, the right format is; wpN: [Lat, Long, Alt, yaw(in radians)] with double types");
        throw rclcpp::exceptions::InvalidParametersException(
                "[ERROR] See above error"
                " the right format is; wpN: [Lat, Long, Alt, yaw(in radians)] with double types"
                "E.g gps_waypoint0; [0.0, 0.0, 0.0, 0.0], please chechk YAML file");
      }
      // construct the gps waypoint and push them to their vector
      // lat, long , alt
      geographic_msgs::msg::GeoPose gps_pose;
      gps_pose.position.latitude = gps_waypoint_vector.at(0);
      gps_pose.position.longitude = gps_waypoint_vector.at(1);
      gps_pose.position.altitude = gps_waypoint_vector.at(2);
      tf2::Quaternion gps_pose_quat;
      gps_pose_quat.setRPY(0.0, 0.0, gps_waypoint_vector.at(3));
      gps_pose.orientation = tf2::toMsg(gps_pose_quat);
      gps_waypoint_msg_vector.push_back(gps_pose);
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }
  }
  // return the read pair of this gps waypoint to it's caller
  return gps_waypoint_msg_vector;
}

void GPSWayPointFollowerClient::goalResponseCallback(
  GPSWaypointFollowerGoalHandle::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void GPSWayPointFollowerClient::feedbackCallback(
  GPSWaypointFollowerGoalHandle::SharedPtr,
  const std::shared_ptr<const ClientT::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Current waypoint: %i", feedback->current_waypoint);
}

void GPSWayPointFollowerClient::resultCallback(
  const GPSWaypointFollowerGoalHandle::WrappedResult & result)
{
  this->goal_done_ = true;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received");
  for (auto number : result.result->missed_waypoints) {
    RCLCPP_INFO(this->get_logger(), "Missed Waypoint %i", number.index);
  }
}

bool GPSWayPointFollowerClient::is_goal_done() const
{
  return this->goal_done_;
}

}  // namespace nav2_gps_waypoint_follower_demo

/**
 * @brief Entry point for Way Point following demo Node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto gps_waypoint_follower_client_node = std::make_shared
    <nav2_gps_waypoint_follower_demo::GPSWayPointFollowerClient>();

  rclcpp::spin(gps_waypoint_follower_client_node);
  rclcpp::shutdown();
  return 0;
}
