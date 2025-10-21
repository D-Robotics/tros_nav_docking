// Copyright (c) 2024 Open Navigation LLC
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

#include <fstream>
#include "angles/angles.h"
#include "opennav_docking/docking_server.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace opennav_docking
{

DockingServer::DockingServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("docking_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());

  declare_parameter("controller_frequency", 50.0);
  declare_parameter("initial_perception_timeout", 5.0);
  declare_parameter("wait_charge_timeout", 5.0);
  declare_parameter("dock_approach_timeout", 30.0);
  declare_parameter("rotate_to_dock_timeout", 10.0);
  declare_parameter("undock_linear_tolerance", 0.05);
  declare_parameter("undock_angular_tolerance", 0.05);
  declare_parameter("max_retries", 3);
  declare_parameter("base_frame", "base_link");
  declare_parameter("fixed_frame", "odom");
  declare_parameter("dock_backwards", false);
  declare_parameter("dock_prestaging_tolerance", 0.5);
  declare_parameter("odom_topic", "odom");
  declare_parameter("rotation_angular_tolerance", 0.05);
  declare_parameter("backward_projection", 0.25);
  declare_parameter("rotate_to_dock", false);
}

nav2_util::CallbackReturn
DockingServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("initial_perception_timeout", initial_perception_timeout_);
  get_parameter("wait_charge_timeout", wait_charge_timeout_);
  get_parameter("dock_approach_timeout", dock_approach_timeout_);
  get_parameter("rotate_to_dock_timeout", rotate_to_dock_timeout_);
  get_parameter("undock_linear_tolerance", undock_linear_tolerance_);
  get_parameter("undock_angular_tolerance", undock_angular_tolerance_);
  get_parameter("max_retries", max_retries_);
  get_parameter("base_frame", base_frame_);
  get_parameter("fixed_frame", fixed_frame_);
  get_parameter("dock_backwards", dock_backwards_);
  get_parameter("dock_prestaging_tolerance", dock_prestaging_tolerance_);
  get_parameter("rotation_angular_tolerance", rotation_angular_tolerance_);
  get_parameter("backward_projection", backward_projection_);

  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create odom subscriber for backward blind docking
  std::string odom_topic;
  get_parameter("odom_topic", odom_topic);
  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node, odom_topic);

  get_parameter("rotate_to_dock", rotate_to_dock_);
  if (rotate_to_dock_ && !dock_backwards_) {
    throw std::runtime_error{"Parameter rotate_to_dock is enabled but dock_backwards is not set."
            "Please set dock_backwards to true."};
  }

  double action_server_result_timeout;
  nav2_util::declare_parameter_if_not_declared(
    node, "action_server_result_timeout", rclcpp::ParameterValue(10.0));
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  RCLCPP_WARN(get_logger(),
    "\n\t      dock_charged_pose_topic: %s" \
    "\n\t     docking_start_stop_topic: %s" \
    "\n\t   undocking_start_stop_topic: %s" \
    "\n\t         docking_status_topic: %s"
    "\n\t         controller_frequency: %.2f" \
    "\n\t   initial_perception_timeout: %.2f" \
    "\n\t          wait_charge_timeout: %.2f" \
    "\n\t        dock_approach_timeout: %.2f" \
    "\n\t       rotate_to_dock_timeout: %.2f" \
    "\n\t      undock_linear_tolerance: %.2f" \
    "\n\t     undock_angular_tolerance: %.2f" \
    "\n\t                  max_retries: %d" \
    "\n\t                   base_frame: %s" \
    "\n\t                  fixed_frame: %s" \
    "\n\t               dock_backwards: %s" \
    "\n\t    dock_prestaging_tolerance: %.2f" \
    "\n\t   rotation_angular_tolerance: %.2f" \
    "\n\t          backward_projection: %.2f"
    "\n\t                   odom_topic: %s" \
    "\n\t               rotate_to_dock: %s" \
    "\n\t action_server_result_timeout: %.2f",
    dock_charged_pose_topic_.data(),
    docking_start_stop_topic_.data(),
    undocking_start_stop_topic_.data(),
    docking_status_topic_.data(),
    controller_frequency_,
    initial_perception_timeout_,
    wait_charge_timeout_,
    dock_approach_timeout_,
    rotate_to_dock_timeout_,
    undock_linear_tolerance_,
    undock_angular_tolerance_,
    max_retries_,
    base_frame_.data(),
    fixed_frame_.data(),
    (dock_backwards_ ? "true" : "false"),
    dock_prestaging_tolerance_,
    rotation_angular_tolerance_,
    backward_projection_,
    odom_topic.data(),
    (rotate_to_dock_ ? "true" : "false"),
    action_server_result_timeout
  );

  dock_charged_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    dock_charged_pose_topic_, rclcpp::QoS(1).transient_local());
  
  pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&DockingServer::publishDockChargedPose, this));

  // Create the action servers for dock / undock
  docking_action_server_ = std::make_unique<DockingActionServer>(
    node, "dock_robot",
    std::bind(&DockingServer::dockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  undocking_action_server_ = std::make_unique<UndockingActionServer>(
    node, "undock_robot",
    std::bind(&DockingServer::undockRobot, this),
    // nullptr,
    [this](){
      RCLCPP_INFO(get_logger(), "completion callback");
      publishZeroVelocity();
    },
    std::chrono::milliseconds(500),
    true, server_options);

  undocking_start_stop_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    undocking_start_stop_topic_,
    rclcpp::QoS(1).transient_local(), std::bind(&DockingServer::undockingStartStopCallback, this, _1));
  docking_start_stop_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    docking_start_stop_topic_,
    rclcpp::QoS(1).transient_local(), std::bind(&DockingServer::dockingStartStopCallback, this, _1));
  docking_status_pub_ = create_publisher<std_msgs::msg::String>(
    docking_status_topic_, rclcpp::QoS(1).transient_local());

  // Create composed utilities
  mutex_ = std::make_shared<std::mutex>();
  controller_ = std::make_unique<Controller>(node, tf2_buffer_, fixed_frame_, base_frame_);
  navigator_ = std::make_unique<Navigator>(node);
  dock_db_ = std::make_unique<DockDatabase>(mutex_);
  if (!dock_db_->initialize(node, tf2_buffer_)) {
    return nav2_util::CallbackReturn::FAILURE;
  }
  dock_ = std::make_shared<Dock>();
  dock_->plugin = dock_db_->findDockPlugin("");

  loadDockPose();

# if 1
  static rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
    1s, [this]() {
      geometry_msgs::msg::PoseStamped robot_pose = getRobotPoseInFrame("map");
      if (dock_ && dock_->plugin->isCharging()) {
        if (current_docking_state_ != DockingState::DOCKING ||
            fabs(angles::shortest_angular_distance(
              tf2::getYaw(initial_dock_pose_.pose.orientation),
              tf2::getYaw(robot_pose.pose.orientation))) > angles::from_degrees(5) ||
              std::hypot(initial_dock_pose_.pose.position.x - robot_pose.pose.position.x,
                initial_dock_pose_.pose.position.y - robot_pose.pose.position.y) > 0.02
        ) {
          RCLCPP_WARN(get_logger(),
            "Update dock pose:" \
            "\n\t frame_id: '%s'" \
            "\n\t pose: {position: {x: %.3f, y: %.3f, z: %.3f}, orientation: {x: %.3f, y: %.3f, z: %.3f, w: %.3f}}",
            robot_pose.header.frame_id.c_str(),
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
            robot_pose.pose.position.z,
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w
          );

          // update dock pose
          initial_dock_pose_ = robot_pose;
          saveDockPose();
        }
        current_docking_state_ = DockingState::DOCKING;
      } else {
        current_docking_state_ = DockingState::UNDOCKING;
      }
      
      RCLCPP_DEBUG(get_logger(),
        "robot_pose:" \
        "\n\t frame_id: '%s'" \
        "\n\t pose: {position: {x: %.3f, y: %.3f, z: %.3f}, orientation: {x: %.3f, y: %.3f, z: %.3f, w: %.3f}}",
        robot_pose.header.frame_id.c_str(),
        robot_pose.pose.position.x,
        robot_pose.pose.position.y,
        robot_pose.pose.position.z,
        robot_pose.pose.orientation.x,
        robot_pose.pose.orientation.y,
        robot_pose.pose.orientation.z,
        robot_pose.pose.orientation.w
      );
    }
  );
#endif 

  if (enable_diag_) {
    diag_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diag_topic_name_, 5);
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());

  auto node = shared_from_this();

  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
  dock_db_->activate();
  navigator_->activate();
  vel_publisher_->on_activate();
  docking_action_server_->activate();
  undocking_action_server_->activate();
  curr_dock_type_.clear();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&DockingServer::dynamicParametersCallback, this, _1));

  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  docking_action_server_->deactivate();
  undocking_action_server_->deactivate();
  dock_db_->deactivate();
  navigator_->deactivate();
  vel_publisher_->on_deactivate();

  dyn_params_handler_.reset();
  tf2_listener_.reset();

  // Destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  tf2_buffer_.reset();
  docking_action_server_.reset();
  undocking_action_server_.reset();
  dock_db_.reset();
  navigator_.reset();
  curr_dock_type_.clear();
  controller_.reset();
  vel_publisher_.reset();
  odom_sub_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename ActionT>
void DockingServer::getPreemptedGoalIfRequested(
  typename std::shared_ptr<const typename ActionT::Goal> goal,
  const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (action_server && action_server->is_server_active() &&
    action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfCancelled(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server && action_server->is_server_active() &&
    action_server->is_cancel_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was cancelled. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfPreempted(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server && action_server->is_server_active() &&
    action_server->is_preempt_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was preempted. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

void DockingServer::dockingStartStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "docking start stop callback with data: %s",
    (msg->data ? "true" : "false"));
  current_docking_request_type_ = DockingRequestType::TOPIC;
  {
    std::lock_guard<std::mutex> lock(docking_mutex_);
    if (msg->data) {
      if (docking_action_state_ == DockingActionState::DOCKING) {
        return;
      } else {
        // async start docking
        RCLCPP_INFO(get_logger(), "docking start");
        docking_action_state_ = DockingActionState::DOCKING;
        
        if (docking_thread_ && docking_thread_->joinable()) {
          docking_continue_ = false;
          docking_thread_->join();
        }
        docking_continue_ = true;
        docking_thread_ = std::make_unique<std::thread>([this]() {
          RCLCPP_INFO(get_logger(), "docking thread start");
          publishDockingStatus("docking start");
          doDocking();
          docking_action_state_ = DockingActionState::IDLE;
          RCLCPP_INFO(get_logger(), "docking thread stop");
          publishDockingStatus("docking completed");
        });
      }
    } else {
      if (docking_action_state_ == DockingActionState::DOCKING) {
        // async stop docking
        RCLCPP_INFO(get_logger(), "docking stop");
        docking_continue_ = false;
        publishDockingStatus("cancel docking");
        navigator_->cancel();
        docking_action_state_ = DockingActionState::IDLE;
        publishDockingStatus("cancel completed");
      } else {
        return;
      }
    }
  }

}

void DockingServer::undockingStartStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "undocking start stop callback with data: %s",
    (msg->data ? "true" : "false"));
  current_docking_request_type_ = DockingRequestType::TOPIC;
  {
    std::lock_guard<std::mutex> lock(docking_mutex_);
    if (msg->data) {
      // start
      if (docking_action_state_ != DockingActionState::IDLE) {
        publishDockingStatus("invalid docking state");
        return;
      } else {
        // async start undocking
        RCLCPP_INFO(get_logger(), "undocking start");
        docking_action_state_ = DockingActionState::UNDOCKING;

        if (docking_thread_ && docking_thread_->joinable()) {
          docking_continue_ = false;
          docking_thread_->join();
        }
        docking_continue_ = true;
        docking_thread_ = std::make_unique<std::thread>([this]() {
          RCLCPP_INFO(get_logger(), "undocking thread start");
          publishDockingStatus("undocking start");
          doUndocking();
          docking_action_state_ = DockingActionState::IDLE;
          RCLCPP_INFO(get_logger(), "undocking thread stop");
          publishDockingStatus("undocking completed");
        });
      }
    } else {
      // stop
      if (docking_action_state_ == DockingActionState::UNDOCKING) {
        // async stop undocking
        RCLCPP_INFO(get_logger(), "cancel undocking");
        docking_continue_ = false;
        docking_action_state_ = DockingActionState::IDLE;
        publishDockingStatus("cancel completed");
      } else {
        return;
      }
    }
  }
}

void DockingServer::doDocking() {
  publishDiagnostics("Moving to dock");

  std::lock_guard<std::mutex> lock(*mutex_);

  auto result = std::make_shared<DockRobot::Result>();
  result->success = false;

  dock_->frame = initial_dock_pose_.header.frame_id;
  dock_->pose = initial_dock_pose_.pose;
  auto dock = dock_.get();

  num_retries_ = 0;
  bool use_dock_id = false;

  try {
    // Send robot to its staging pose
    publishDockingFeedback(DockRobot::Feedback::NAV_TO_STAGING_POSE);
    const auto initial_staging_pose = dock->getStagingPose();
    const auto robot_pose = getRobotPoseInFrame(initial_staging_pose.header.frame_id);
    if (!navigate_to_staging_pose_ ||
      utils::l2Norm(robot_pose.pose, initial_staging_pose.pose) < dock_prestaging_tolerance_)
    {
      RCLCPP_INFO(get_logger(), "Robot already within pre-staging pose tolerance for dock");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "initial_staging_pose (%0.2f, %0.2f), dist of robot with initial staging_pose: %.2f, navigation to staging pose.",
        initial_staging_pose.pose.position.x, initial_staging_pose.pose.position.y,
        nav2_util::geometry_utils::euclidean_distance(initial_staging_pose, robot_pose)
      );

      navigator_->goToPose(
        initial_staging_pose, rclcpp::Duration::from_seconds(max_staging_time_));
      RCLCPP_INFO(get_logger(), "Successful navigation to staging pose");
    }

    RCLCPP_INFO(get_logger(), "Staging dock robot success");
    
    // Construct initial estimate of where the dock is located in fixed_frame
    auto dock_pose = utils::getDockPoseStamped(dock, rclcpp::Time(0));
    tf2_buffer_->transform(dock_pose, dock_pose, fixed_frame_);

    // Get initial detection of dock before proceeding to move
    doInitialPerception(dock, dock_pose);
    RCLCPP_INFO(get_logger(),
      "Successful initial dock detection with frame id: %s",
      dock_pose.header.frame_id.data());

    // If we performed a rotation before docking backward, we must rotate the staging pose
    // to match the robot orientation
    auto staging_pose = dock->getStagingPose();
    if (rotate_to_dock_) {
      staging_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(staging_pose.pose.orientation) + M_PI);
    }

    // Docking control loop: while not docked, run controller
    rclcpp::Time dock_contact_time;
    while (rclcpp::ok()) {
      if (!docking_continue_)
      {
        publishZeroVelocity();
        return;
      }

      try {
        // Perform a 180º to face away from the dock if needed
        if (rotate_to_dock_) {
          rotateToDock(dock_pose);
        }
        // Approach the dock using control law
        if (approachDock(dock, dock_pose)) {
          // We are docked, wait for charging to begin
          RCLCPP_INFO(get_logger(), "Made contact with dock, waiting for charge to start");
          if (waitForCharge(dock)) {
            RCLCPP_INFO(get_logger(), "Robot is charging!");
            result->success = true;
            result->num_retries = num_retries_;
            stashDockData(use_dock_id, dock, true);
            publishZeroVelocity();
            return;
          }
        }

        // Cancelled, preempted, or shutting down (recoverable errors throw DockingException)
        stashDockData(use_dock_id, dock, false);
        publishZeroVelocity();
        return;
      } catch (opennav_docking_core::DockingException & e) {
        if (++num_retries_ > max_retries_) {
          RCLCPP_ERROR(get_logger(), "Failed to dock, all retries have been used");
          throw;
        }
        RCLCPP_WARN(get_logger(), "Docking failed, will retry: %s", e.what());
      }

      // Reset to staging pose to try again
      if (!resetApproach(staging_pose)) {
        // Cancelled, preempted, or shutting down
        stashDockData(use_dock_id, dock, false);
        publishZeroVelocity();
        return;
      }
      RCLCPP_INFO(get_logger(), "Returned to staging pose, attempting docking again");
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (opennav_docking_core::DockNotInDB & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_IN_DB;
  } catch (opennav_docking_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToStage & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_STAGE;
  } catch (opennav_docking_core::FailedToDetectDock & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_DETECT_DOCK;
  } catch (opennav_docking_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::FailedToCharge & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CHARGE;
  } catch (opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  // Store dock state for later undocking and delete temp dock, if applicable
  stashDockData(use_dock_id, dock, false);
  result->num_retries = num_retries_;
  publishZeroVelocity();
}

void DockingServer::doUndocking() {
  publishDiagnostics("Moving away from dock");

  std::lock_guard<std::mutex> lock(*mutex_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto result = std::make_shared<UndockRobot::Result>();
  result->success = false;

  auto max_duration = rclcpp::Duration::from_seconds(max_undocking_time_);

  try {
    // Get dock plugin information from request or docked state, reset state.
    std::string dock_type = curr_dock_type_;

    ChargingDock::Ptr dock = dock_db_->findDockPlugin(dock_type);
    if (!dock) {
      throw opennav_docking_core::DockNotValid("No dock information to undock from!");
    }
    RCLCPP_INFO(
      get_logger(),
      "Attempting to undock robot from charger of type %s.", dock->getName().c_str());

    if (!dock->isCharging()) {
      RCLCPP_WARN(
        get_logger(),
        "Robot is not charging, do not need to undock.");
      return;
    }

    // Get "dock pose" by finding the robot pose
    geometry_msgs::msg::PoseStamped dock_pose = getRobotPoseInFrame(fixed_frame_);

    // Make sure that the staging pose is pointing in the same direction when moving backwards
    if (dock_backwards_) {
      dock_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(dock_pose.pose.orientation) + M_PI);
    }

    // Get staging pose (in fixed frame)
    geometry_msgs::msg::PoseStamped staging_pose =
      dock->getStagingPose(dock_pose.pose, dock_pose.header.frame_id);

    // If we performed a rotation before docking backward, we must rotate the staging pose
    // to match the robot orientation
    if (rotate_to_dock_) {
      staging_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(staging_pose.pose.orientation) + M_PI);
    }

    // Control robot to staging pose
    rclcpp::Time loop_start = this->now();
    while (rclcpp::ok()) {
      // Stop if we exceed max duration
      auto timeout = max_undocking_time_;
      if (this->now().seconds() - loop_start.seconds() > timeout) {
        throw opennav_docking_core::FailedToControl("Undocking timed out");
      }

      // Stop if cancelled/preempted
      if (!docking_continue_)
      {
        RCLCPP_INFO(get_logger(), "canceled");
        publishZeroVelocity();
        return;
      }

      // Don't control the robot until charging is disabled
      if (!dock->disableCharging()) {
        loop_rate.sleep();
        continue;
      }

      geometry_msgs::msg::PoseStamped robot_pose = getRobotPoseInFrame(staging_pose.header.frame_id);
      auto dist = nav2_util::geometry_utils::euclidean_distance(robot_pose, staging_pose);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "dist of robot with staging_pose: %.2f", dist);
        
      // Get command to approach staging pose
      geometry_msgs::msg::Twist command;
      if (getCommandToPose(
          command, staging_pose, undock_linear_tolerance_, undock_angular_tolerance_, false,
          !dock_backwards_))
      {
        // Perform a 180º to the original staging pose
        if (rotate_to_dock_) {
          rotateToDock(staging_pose);
        }

        // Have reached staging_pose
        RCLCPP_INFO(get_logger(), "Robot has reached staging pose");
        vel_publisher_->publish(command);
        if (dock->hasStoppedCharging()) {
          RCLCPP_INFO(get_logger(), "Robot has undocked!");
          result->success = true;
          curr_dock_type_.clear();
          publishZeroVelocity();
          return;
        }
        // Haven't stopped charging?
        throw opennav_docking_core::FailedToControl("Failed to control off dock, still charging");
      }

      command.angular.z = 0;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "vel command: %.2f %.2f %.2f",
        command.linear.x, command.linear.y, command.angular.z);
        
      // Publish command and sleep
      vel_publisher_->publish(command);
      loop_rate.sleep();
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (opennav_docking_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Internal error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  publishZeroVelocity();
}

void DockingServer::dockRobot()
{
  publishDiagnostics("Moving to dock");
  current_docking_request_type_ = DockingRequestType::ACTION;

  std::lock_guard<std::mutex> lock(*mutex_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = docking_action_server_->get_current_goal();
  auto result = std::make_shared<DockRobot::Result>();
  result->success = false;

  if (!docking_action_server_ || !docking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot")) {
    docking_action_server_->terminate_all();
    return;
  }

  getPreemptedGoalIfRequested(goal, docking_action_server_);
  Dock * dock{nullptr};
  num_retries_ = 0;

  try {
    // Get dock (instance and plugin information) from request
    if (goal->use_dock_id) {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at charger %s.", goal->dock_id.c_str());
      dock = dock_db_->findDock(goal->dock_id);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at charger at position (%0.2f, %0.2f).",
        goal->dock_pose.pose.position.x, goal->dock_pose.pose.position.y);
      dock = generateGoalDock(goal);
    }

    // Send robot to its staging pose
    publishDockingFeedback(DockRobot::Feedback::NAV_TO_STAGING_POSE);
    const auto initial_staging_pose = dock->getStagingPose();
    const auto robot_pose = getRobotPoseInFrame(initial_staging_pose.header.frame_id);
    if (!goal->navigate_to_staging_pose ||
      utils::l2Norm(robot_pose.pose, initial_staging_pose.pose) < dock_prestaging_tolerance_)
    {
      RCLCPP_INFO(get_logger(), "Robot already within pre-staging pose tolerance for dock");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "initial_staging_pose (%0.2f, %0.2f).",
        initial_staging_pose.pose.position.x, initial_staging_pose.pose.position.y);

      // navigator_->goToPose(
      //   initial_staging_pose, rclcpp::Duration::from_seconds(goal->max_staging_time));
      RCLCPP_INFO(get_logger(), "Successful navigation to staging pose");
    }

    RCLCPP_INFO(get_logger(), "Dock robot success");

    
    // Construct initial estimate of where the dock is located in fixed_frame
    auto dock_pose = utils::getDockPoseStamped(dock, rclcpp::Time(0));
    tf2_buffer_->transform(dock_pose, dock_pose, fixed_frame_);

    // Get initial detection of dock before proceeding to move
    doInitialPerception(dock, dock_pose);
    RCLCPP_INFO(get_logger(), "Successful initial dock detection");

    // If we performed a rotation before docking backward, we must rotate the staging pose
    // to match the robot orientation
    auto staging_pose = dock->getStagingPose();
    if (rotate_to_dock_) {
      staging_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(staging_pose.pose.orientation) + M_PI);
    }

    // Docking control loop: while not docked, run controller
    rclcpp::Time dock_contact_time;
    while (rclcpp::ok()) {
      try {
        // Perform a 180º to face away from the dock if needed
        if (rotate_to_dock_) {
          rotateToDock(dock_pose);
        }
        // Approach the dock using control law
        if (approachDock(dock, dock_pose)) {
          // We are docked, wait for charging to begin
          RCLCPP_INFO(get_logger(), "Made contact with dock, waiting for charge to start");
          if (waitForCharge(dock)) {
            RCLCPP_INFO(get_logger(), "Robot is charging!");
            result->success = true;
            result->num_retries = num_retries_;
            stashDockData(goal->use_dock_id, dock, true);
            publishZeroVelocity();
            docking_action_server_->succeeded_current(result);
            return;
          }
        }

        // Cancelled, preempted, or shutting down (recoverable errors throw DockingException)
        stashDockData(goal->use_dock_id, dock, false);
        publishZeroVelocity();
        docking_action_server_->terminate_all(result);
        return;
      } catch (opennav_docking_core::DockingException & e) {
        if (++num_retries_ > max_retries_) {
          RCLCPP_ERROR(get_logger(), "Failed to dock, all retries have been used");
          throw;
        }
        RCLCPP_WARN(get_logger(), "Docking failed, will retry: %s", e.what());
      }

      // Reset to staging pose to try again
      if (!resetApproach(staging_pose)) {
        // Cancelled, preempted, or shutting down
        stashDockData(goal->use_dock_id, dock, false);
        publishZeroVelocity();
        docking_action_server_->terminate_all(result);
        return;
      }
      RCLCPP_INFO(get_logger(), "Returned to staging pose, attempting docking again");
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (opennav_docking_core::DockNotInDB & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_IN_DB;
  } catch (opennav_docking_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToStage & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_STAGE;
  } catch (opennav_docking_core::FailedToDetectDock & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_DETECT_DOCK;
  } catch (opennav_docking_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::FailedToCharge & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CHARGE;
  } catch (opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  // Store dock state for later undocking and delete temp dock, if applicable
  stashDockData(goal->use_dock_id, dock, false);
  result->num_retries = num_retries_;
  publishZeroVelocity();
  docking_action_server_->terminate_current(result);
}

void DockingServer::stashDockData(bool use_dock_id, Dock * dock, bool successful)
{
  return;

  if (dock && successful) {
    curr_dock_type_ = dock->type;
  }

  if (!use_dock_id && dock) {
    delete dock;
    dock = nullptr;
  }
}

Dock * DockingServer::generateGoalDock(std::shared_ptr<const DockRobot::Goal> goal)
{
  auto dock = new Dock();
  dock->frame = goal->dock_pose.header.frame_id;
  dock->pose = goal->dock_pose.pose;
  dock->type = goal->dock_type;
  dock->plugin = dock_db_->findDockPlugin(dock->type);
  return dock;
}

void DockingServer::doInitialPerception(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  publishDockingFeedback(DockRobot::Feedback::INITIAL_PERCEPTION);
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(initial_perception_timeout_);
  while (!dock->plugin->getRefinedPose(dock_pose)) {
    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToDetectDock("Failed initial dock detection");
    }

    // check for docking request is running or not
    if (current_docking_request_type_ == DockingRequestType::ACTION) {
      if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
        checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
      {
        return;
      }
    }

    loop_rate.sleep();
  }
}

void DockingServer::rotateToDock(const geometry_msgs::msg::PoseStamped & dock_pose)
{
  const double dt = 1.0 / controller_frequency_;
  auto target_pose = dock_pose;
  target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
    tf2::getYaw(target_pose.pose.orientation) + M_PI);

  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(rotate_to_dock_timeout_);

  while (rclcpp::ok()) {
    auto robot_pose = getRobotPoseInFrame(dock_pose.header.frame_id);
    auto angular_distance_to_heading = angles::shortest_angular_distance(
      tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(target_pose.pose.orientation));
    if (fabs(angular_distance_to_heading) < rotation_angular_tolerance_) {
      break;
    }

    geometry_msgs::msg::Twist current_vel;
    current_vel.angular.z = odom_sub_->getTwist().theta;

    auto command = controller_->computeRotateToHeadingCommand(
      angular_distance_to_heading, current_vel, dt);

    vel_publisher_->publish(command);

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToControl("Timed out rotating to dock");
    }

    loop_rate.sleep();
  }
}

bool DockingServer::approachDock(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(dock_approach_timeout_);
  
  RCLCPP_INFO(get_logger(),
    "dock_approach_timeout: %.2f, dock_pose frame_id: %s",
    dock_approach_timeout_,
    dock_pose.header.frame_id.data()
  );
  
  geometry_msgs::msg::PoseStamped last_robot_pose = getRobotPoseInFrame(dock_pose.header.frame_id);

  bool rotate_to_approach = false;

  while (rclcpp::ok()) {
    if (!docking_continue_)
    {
      RCLCPP_INFO(get_logger(), "canceled");
      publishZeroVelocity();
      return false;
    }
    
    // Stop and report success if connected to dock
    if (dock->plugin->isDocked() || dock->plugin->isCharging()) {
      RCLCPP_INFO(get_logger(),
        "robot is docked or in charging");
      publishZeroVelocity();
      return true;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Approaching dock");
    publishDockingFeedback(DockRobot::Feedback::CONTROLLING);

    // Stop if cancelled/preempted
    if (current_docking_request_type_ == DockingRequestType::ACTION) {
      if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
        checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
      {
        return false;
      }
    }

    // Update perception
    if (!dock->plugin->getRefinedPose(dock_pose) && !rotate_to_dock_) {
      throw opennav_docking_core::FailedToDetectDock("Failed dock detection");
    }

    geometry_msgs::msg::PoseStamped robot_pose = getRobotPoseInFrame(dock_pose.header.frame_id);

    if (rotate_to_approach) {
      auto command = rotateToApproach(dock_pose, robot_pose,
        0.03, 3.0, angles::from_degrees(10));
      // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100,
      RCLCPP_INFO(get_logger(),
        "rotate_to_approach" \
        "\n vel command: %.2f %.2f %.2f",
        command.linear.x, command.linear.y, command.angular.z);
      vel_publisher_->publish(command);
      if (this->now() - start > timeout) {
        RCLCPP_WARN(get_logger(), "approach dock timeout: %.2f", dock_approach_timeout_);
        throw opennav_docking_core::FailedToControl(
                "Timed out approaching dock; dock nor charging detected");
      }
      loop_rate.sleep();
      continue;
    }
    
    auto dist = nav2_util::geometry_utils::euclidean_distance(robot_pose, last_robot_pose);
    double dyaw = angles::shortest_angular_distance(
      tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(last_robot_pose.pose.orientation));
    if (dist >= 0.01 || abs(dyaw) >= 10.05) {
      // robot is moving, update last_robot_pose
      last_robot_pose = robot_pose;
    }

    // Transform target_pose into base_link frame
    geometry_msgs::msg::PoseStamped target_pose = dock_pose;
    target_pose.header.stamp = rclcpp::Time(0);

    // The control law can get jittery when close to the end when atan2's can explode.
    // Thus, we backward project the controller's target pose a little bit after the
    // dock so that the robot never gets to the end of the spiral before its in contact
    // with the dock to stop the docking procedure.
    // const double backward_projection = 0.25;
    const double yaw = tf2::getYaw(target_pose.pose.orientation);
    target_pose.pose.position.x += cos(yaw) * backward_projection_;
    target_pose.pose.position.y += sin(yaw) * backward_projection_;
    tf2_buffer_->transform(target_pose, target_pose, base_frame_);

    // Make sure that the target pose is pointing at the robot when moving backwards
    // This is to ensure that the robot doesn't try to dock from the wrong side
    if (dock_backwards_) {
      target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(target_pose.pose.orientation) + M_PI);
    }

    // Compute and publish controls
    geometry_msgs::msg::Twist command;
    if (!controller_->computeVelocityCommand(target_pose.pose, command, true, dock_backwards_)) {
      throw opennav_docking_core::FailedToControl("Failed to get control");
    }
    
    dist = nav2_util::geometry_utils::euclidean_distance(robot_pose, dock_pose);
    RCLCPP_INFO(get_logger(),
      "\n robot with dock dist: %.2f" \
      "\n vel command: %.2f %.2f %.2f",
      dist,
      command.linear.x, command.linear.y, command.angular.z);

    // 如果robot和dock之间距离小于阈值，且robot长时间位置没有变化，控制robot只旋转
    // if (dist <= 0.05 &&
    //     (rclcpp::Time(robot_pose.header.stamp).seconds() - rclcpp::Time(last_robot_pose.header.stamp).seconds() >= 1)) {
    //   RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    //     "robot and dock distance is small and robot did not move for more than 1 sec, only rotate");
    //   command.linear.x = 0;
    //   if (command.angular.z > 0.03) {
    //     command.angular.z = 0.03;
    //   }
    //   if (command.angular.z < -0.03) {
    //     command.angular.z = -0.03;
    //   }
    //   // TODO: rotating controller
    //   command = rotateToApproach(dock_pose, robot_pose,
    //     0.03, 3.0, angles::from_degrees(30));
    // }
    float time_thr = 2;
    if (
        (rclcpp::Time(robot_pose.header.stamp).seconds() -
        rclcpp::Time(last_robot_pose.header.stamp).seconds() >= time_thr)) {
      // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100,
      //   "robot did not move for more than 1 sec, only rotate");
      RCLCPP_WARN(get_logger(),
        "robot did not move for more than %.2f sec, only rotate", time_thr);
      // rotating controller
      command = rotateToApproach(dock_pose, robot_pose,
        0.03, 3.0, angles::from_degrees(10));
      rotate_to_approach = true;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100,
      "vel command: %.2f %.2f %.2f",
      command.linear.x, command.linear.y, command.angular.z);
    vel_publisher_->publish(command);

    if (this->now() - start > timeout) {
      RCLCPP_WARN(get_logger(), "approach dock timeout: %.2f", dock_approach_timeout_);
      throw opennav_docking_core::FailedToControl(
              "Timed out approaching dock; dock nor charging detected");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::waitForCharge(Dock * dock)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(wait_charge_timeout_);
  auto uncharging_tp = this->now();
  while (rclcpp::ok()) {
    publishDockingFeedback(DockRobot::Feedback::WAIT_FOR_CHARGE);

    if (dock->plugin->isCharging()) {
      if (this->now().seconds() - uncharging_tp.seconds() >= 0.5) {
        RCLCPP_INFO(get_logger(), "Robot is charging for 0.5 seconds");
        return true;
      } else {
        loop_rate.sleep();
        continue;
      }
    } else {
      uncharging_tp = this->now();
    }

    if (current_docking_request_type_ == DockingRequestType::ACTION) {
      if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
        checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
      {
        return false;
      }
    }

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToCharge("Timed out waiting for charge to start");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::resetApproach(const geometry_msgs::msg::PoseStamped & staging_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(dock_approach_timeout_);
  while (rclcpp::ok()) {
    publishDockingFeedback(DockRobot::Feedback::INITIAL_PERCEPTION);

    if (current_docking_request_type_ == DockingRequestType::ACTION) {
      // Stop if cancelled/preempted
      if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
        checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
      {
        return false;
      }
    }

    // Compute and publish command
    geometry_msgs::msg::Twist command;
    if (getCommandToPose(
        command, staging_pose, undock_linear_tolerance_, undock_angular_tolerance_, false,
        !dock_backwards_))
    {
      return true;
    }
    vel_publisher_->publish(command);

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToControl("Timed out resetting dock approach");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::getCommandToPose(
  geometry_msgs::msg::Twist & cmd, const geometry_msgs::msg::PoseStamped & pose,
  double linear_tolerance, double angular_tolerance, bool is_docking, bool backward)
{
  // Reset command to zero velocity
  cmd.linear.x = 0;
  cmd.angular.z = 0;

  // Determine if we have reached pose yet & stop
  geometry_msgs::msg::PoseStamped robot_pose = getRobotPoseInFrame(pose.header.frame_id);
  const double dist =
    nav2_util::geometry_utils::euclidean_distance(robot_pose, pose);
  const double yaw = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(pose.pose.orientation));
  if (dist < linear_tolerance && abs(yaw) < angular_tolerance) {
    return true;
  }

  // Transform target_pose into base_link frame
  geometry_msgs::msg::PoseStamped target_pose = pose;
  target_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(target_pose, target_pose, base_frame_);

  // Compute velocity command
  if (!controller_->computeVelocityCommand(target_pose.pose, cmd, is_docking, backward)) {
    throw opennav_docking_core::FailedToControl("Failed to get control");
  }

  // Command is valid, but target is not reached
  return false;
}

void DockingServer::undockRobot()
{
  publishDiagnostics("Moving away from dock");
  current_docking_request_type_ = DockingRequestType::ACTION;

  std::lock_guard<std::mutex> lock(*mutex_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = undocking_action_server_->get_current_goal();
  auto result = std::make_shared<UndockRobot::Result>();
  result->success = false;

  if (!undocking_action_server_ || !undocking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled(undocking_action_server_, "undock_robot")) {
    undocking_action_server_->terminate_all(result);
    return;
  }

  getPreemptedGoalIfRequested(goal, undocking_action_server_);
  auto max_duration = rclcpp::Duration::from_seconds(goal->max_undocking_time);

  try {
    // Get dock plugin information from request or docked state, reset state.
    std::string dock_type = curr_dock_type_;
    if (!goal->dock_type.empty()) {
      dock_type = goal->dock_type;
    }

    ChargingDock::Ptr dock = dock_db_->findDockPlugin(dock_type);
    if (!dock) {
      throw opennav_docking_core::DockNotValid("No dock information to undock from!");
    }
    RCLCPP_INFO(
      get_logger(),
      "Attempting to undock robot from charger of type %s.", dock->getName().c_str());

    // Get "dock pose" by finding the robot pose
    geometry_msgs::msg::PoseStamped dock_pose = getRobotPoseInFrame(fixed_frame_);

    // Make sure that the staging pose is pointing in the same direction when moving backwards
    if (dock_backwards_) {
      dock_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(dock_pose.pose.orientation) + M_PI);
    }

    // Get staging pose (in fixed frame)
    geometry_msgs::msg::PoseStamped staging_pose =
      dock->getStagingPose(dock_pose.pose, dock_pose.header.frame_id);

    // If we performed a rotation before docking backward, we must rotate the staging pose
    // to match the robot orientation
    if (rotate_to_dock_) {
      staging_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(staging_pose.pose.orientation) + M_PI);
    }

    // Control robot to staging pose
    rclcpp::Time loop_start = this->now();
    while (rclcpp::ok()) {
      // Stop if we exceed max duration
      auto timeout = rclcpp::Duration::from_seconds(goal->max_undocking_time);
      if (this->now() - loop_start > timeout) {
        throw opennav_docking_core::FailedToControl("Undocking timed out");
      }

      // Stop if cancelled/preempted
      if (checkAndWarnIfCancelled(undocking_action_server_, "undock_robot") ||
        checkAndWarnIfPreempted(undocking_action_server_, "undock_robot"))
      {
        publishZeroVelocity();
        undocking_action_server_->terminate_all(result);
        return;
      }

      // Don't control the robot until charging is disabled
      if (!dock->disableCharging()) {
        loop_rate.sleep();
        continue;
      }

      geometry_msgs::msg::PoseStamped robot_pose = getRobotPoseInFrame(staging_pose.header.frame_id);
      auto dist = nav2_util::geometry_utils::euclidean_distance(robot_pose, staging_pose);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "dist of robot with staging_pose: %.2f", dist);
        
      // Get command to approach staging pose
      geometry_msgs::msg::Twist command;
      if (getCommandToPose(
          command, staging_pose, undock_linear_tolerance_, undock_angular_tolerance_, false,
          !dock_backwards_))
      {
        // Perform a 180º to the original staging pose
        if (rotate_to_dock_) {
          rotateToDock(staging_pose);
        }

        // Have reached staging_pose
        RCLCPP_INFO(get_logger(), "Robot has reached staging pose");
        vel_publisher_->publish(command);
        if (dock->hasStoppedCharging()) {
          RCLCPP_INFO(get_logger(), "Robot has undocked!");
          result->success = true;
          curr_dock_type_.clear();
          publishZeroVelocity();
          undocking_action_server_->succeeded_current(result);
          return;
        }
        // Haven't stopped charging?
        throw opennav_docking_core::FailedToControl("Failed to control off dock, still charging");
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "vel command: %.2f %.2f %.2f",
        command.linear.x, command.linear.y, command.angular.z);
        
      // Publish command and sleep
      vel_publisher_->publish(command);
      loop_rate.sleep();
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (opennav_docking_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Internal error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  publishZeroVelocity();
  undocking_action_server_->terminate_current(result);
}

geometry_msgs::msg::PoseStamped DockingServer::getRobotPoseInFrame(const std::string & frame)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame_;
  robot_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(robot_pose, robot_pose, frame);
  return robot_pose;
}

void DockingServer::publishZeroVelocity()
{
  vel_publisher_->publish(geometry_msgs::msg::Twist());
}

void DockingServer::publishDockingFeedback(uint16_t state)
{
  auto feedback = std::make_shared<DockRobot::Feedback>();
  feedback->state = state;
  feedback->docking_time = this->now() - action_start_time_;
  feedback->num_retries = num_retries_;
  
  if (current_docking_request_type_ == DockingRequestType::ACTION) {
    if (!docking_action_server_ || !docking_action_server_->is_server_active()) {
      docking_action_server_->publish_feedback(feedback);
    }
  }

  auto enum2str = [](uint16_t dock_state) -> std::string {
    switch (dock_state) {
      case DockRobot::Feedback::NONE:
        return "NONE";
      case DockRobot::Feedback::NAV_TO_STAGING_POSE:
        return "NAV_TO_STAGING_POSE";
      case DockRobot::Feedback::INITIAL_PERCEPTION:
        return "INITIAL_PERCEPTION";
      case DockRobot::Feedback::CONTROLLING:
        return "CONTROLLING";
      case DockRobot::Feedback::WAIT_FOR_CHARGE:
        return "WAIT_FOR_CHARGE";
      case DockRobot::Feedback::RETRY:
        return "RETRY";
      default:
        return "UNKNOWN";
    }
  };

  publishDockingStatus(enum2str(state));
}

rcl_interfaces::msg::SetParametersResult
DockingServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(*mutex_);

  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "initial_perception_timeout") {
        initial_perception_timeout_ = parameter.as_double();
      } else if (name == "wait_charge_timeout") {
        wait_charge_timeout_ = parameter.as_double();
      } else if (name == "undock_linear_tolerance") {
        undock_linear_tolerance_ = parameter.as_double();
      } else if (name == "undock_angular_tolerance") {
        undock_angular_tolerance_ = parameter.as_double();
      } else if (name == "rotation_angular_tolerance") {
        rotation_angular_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "base_frame") {
        base_frame_ = parameter.as_string();
      } else if (name == "fixed_frame") {
        fixed_frame_ = parameter.as_string();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "max_retries") {
        max_retries_ = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

void DockingServer::publishDockChargedPose() {
  if (dock_charged_pose_pub_) {
    dock_charged_pose_pub_->publish(initial_dock_pose_);
  }
}

void DockingServer::publishDiagnostics(std::string diag_val) {
  if (enable_diag_ && diag_publisher_) {
    // https://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticStatus.html
    auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    msg->header.stamp = this->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = this->get_name();
    status.message = "Robot";
    status.hardware_id = "";
    diagnostic_msgs::msg::KeyValue value;
    value.key = "state";
    value.value = diag_val;
    status.values.push_back(value);
    msg->status.push_back(status);
    diag_publisher_->publish(std::move(msg));
  }
}

void DockingServer::publishDockingStatus(std::string status) {
  if (docking_status_pub_) {
    std_msgs::msg::String str_msg;
    str_msg.data = status;
    docking_status_pub_->publish(str_msg);
  }
}

bool DockingServer::loadDockPose() {
  // load initial dock pose from yaml file
  /* dock.yaml format:
    frame: map
    pose:
      position:
        x: 0.0
        y: 0.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  */ 
  YAML::Node yaml_file;
  try {
    yaml_file = YAML::LoadFile(dock_yaml_filepath_);
  } catch (...) {
    return false;
  }

  initial_dock_pose_.header.frame_id = "map";
  
  if (!yaml_file["frame"]) {
    RCLCPP_ERROR(
      get_logger(),
      "Dock yaml (%s) file do not contain 'frame'.", dock_yaml_filepath_.c_str());
    return false;
  }
  initial_dock_pose_.header.frame_id = yaml_file["frame"].as<std::string>();

  if (!yaml_file["pose"]) {
    RCLCPP_ERROR(
      get_logger(),
      "Dock yaml (%s) file do not contain 'pose'.", dock_yaml_filepath_.c_str());
    return false;
  }
  auto position = yaml_file["pose"]["position"];
  if (!position || !position["x"] || !position["y"] || !position["z"]) {
    RCLCPP_ERROR(
      get_logger(),
      "Dock yaml (%s) file do not contain 'pose'/'xyz'.", dock_yaml_filepath_.c_str());
    return false;
  }
  initial_dock_pose_.pose.position.x = position["x"].as<double>();
  initial_dock_pose_.pose.position.y = position["y"].as<double>();
  initial_dock_pose_.pose.position.z = position["z"].as<double>();
  
  auto orientation = yaml_file["pose"]["orientation"];
  if (!orientation || !orientation["x"] || !orientation["y"] || !orientation["z"] || !orientation["w"]) {
    RCLCPP_ERROR(
      get_logger(),
      "Dock yaml (%s) file do not contain 'orientation'/'xyzw'.", dock_yaml_filepath_.c_str());
    return false;
  }
  initial_dock_pose_.pose.orientation.x = orientation["x"].as<double>();
  initial_dock_pose_.pose.orientation.y = orientation["y"].as<double>();
  initial_dock_pose_.pose.orientation.z = orientation["z"].as<double>();
  initial_dock_pose_.pose.orientation.w = orientation["w"].as<double>();

  RCLCPP_WARN(get_logger(),
    "Load dock pose successfully." \
    "\n  file: %s" \
    "\n frame: %s" \
    "\n     x: %.2f, y: %.2f, z: %.2f" \
    "\n   yaw: %.2f",
    dock_yaml_filepath_.c_str(),
    initial_dock_pose_.header.frame_id.c_str(),
    initial_dock_pose_.pose.position.x, initial_dock_pose_.pose.position.y, initial_dock_pose_.pose.position.z,
    tf2::getYaw(initial_dock_pose_.pose.orientation)
  );

  return true;
}

bool DockingServer::saveDockPose() {
  // save initial_dock_pose_ to yaml
  YAML::Node yaml_file;
  yaml_file["frame"] = initial_dock_pose_.header.frame_id;
  yaml_file["pose"]["position"]["x"] = initial_dock_pose_.pose.position.x;
  yaml_file["pose"]["position"]["y"] = initial_dock_pose_.pose.position.y;
  yaml_file["pose"]["position"]["z"] = initial_dock_pose_.pose.position.z;
  yaml_file["pose"]["orientation"]["x"] = initial_dock_pose_.pose.orientation.x;
  yaml_file["pose"]["orientation"]["y"] = initial_dock_pose_.pose.orientation.y;
  yaml_file["pose"]["orientation"]["z"] = initial_dock_pose_.pose.orientation.z;
  yaml_file["pose"]["orientation"]["w"] = initial_dock_pose_.pose.orientation.w;
  YAML::Emitter out;
  out << yaml_file;
  std::ofstream fout(dock_yaml_filepath_);
  fout << out.c_str();
  fout.close();
  RCLCPP_WARN(get_logger(),
    "Save dock pose successfully." \
    "\n  file: %s" \
    "\n frame: %s" \
    "\n     x: %.2f, y: %.2f, z: %.2f" \
    "\n   yaw: %.2f",
    dock_yaml_filepath_.c_str(),
    initial_dock_pose_.header.frame_id.c_str(),
    initial_dock_pose_.pose.position.x, initial_dock_pose_.pose.position.y, initial_dock_pose_.pose.position.z,
    tf2::getYaw(initial_dock_pose_.pose.orientation)
  );
  return true;
}

geometry_msgs::msg::Twist DockingServer::rotateToApproach(const geometry_msgs::msg::PoseStamped & dock_pose,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  float angular_vel, float max_rotation_time, float yaw_tolerance) {
  double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(dock_pose.pose.orientation),
    tf2::getYaw(robot_pose.pose.orientation));
  auto time_diff = this->now().seconds() - dock_approach_start_time_.seconds();
  if (fabs(dyaw) > yaw_tolerance ||
    time_diff > max_rotation_time) {
    RCLCPP_DEBUG(get_logger(),
      "Rotate to approach dock with new approach direction, dyaw: %.2f, time_diff: %.2f.",
      fabs(dyaw), time_diff
    );
    // dock_approach_wise_ *= -1;
    // dock_approach_start_time_ = this->now();
  }

    RCLCPP_INFO(get_logger(),
      "Rotate to approach dock, dyaw: %.2f, %.2f degrees, yaw_tolerance: %.2f degrees.",
      dyaw, angles::to_degrees(fabs(dyaw)),
      angles::to_degrees(fabs(yaw_tolerance))
    );
  
  if (dyaw < (-1.0) * yaw_tolerance) {
    RCLCPP_INFO(get_logger(),
      "Rotate to approach dock with new approach direction, dyaw: %.2f.",
      fabs(dyaw)
    );
    dock_approach_wise_ = 1;
  } else if (dyaw > yaw_tolerance) {
    RCLCPP_INFO(get_logger(),
      "Rotate to approach dock with new approach direction, dyaw: %.2f.",
      fabs(dyaw)
    );
    dock_approach_wise_ = -1.0;
  } else {
    RCLCPP_INFO(get_logger(),
      "Rotate to approach dock with new approach direction, dyaw: %.2f, %.2f degrees, yaw_tolerance: %.2f degrees, dock_approach_wise: %d.",
      fabs(dyaw), angles::to_degrees(fabs(dyaw)),
      angles::to_degrees(fabs(yaw_tolerance)),
      dock_approach_wise_
    );
  }

  geometry_msgs::msg::Twist command;
  command.linear.x = 0;
  command.linear.y = 0;
  command.linear.z = 0;
  command.angular.x = 0;
  command.angular.y = 0;
  command.angular.z = angular_vel * dock_approach_wise_;
  return command;
}

}  // namespace opennav_docking

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(opennav_docking::DockingServer)
