/*
 * Copyright 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
    This file has been modified from the original, by Devon Ash, then further by Peter Mitrano
*/

#include <ros/ros.h>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <victor_hardware_interface_msgs/Robotiq3FingerCommand.h>
#include <victor_hardware_interface_msgs/Robotiq3FingerStatus.h>
#include <robotiq_3f_gripper_articulated_gazebo_plugins/RobotiqHandPlugin.h>

void RobotiqHandPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Initialize ROS.
  if (!ros::isInitialized())
  {
    ROS_ERROR_STREAM("Not loading plugin since ROS hasn't been properly initialized.");
    return;
  }

  if (!sdf->HasElement("side"))
  {
    ROS_ERROR_STREAM("Failed to determine which hand we're controlling. <side> should be either 'left' or 'right'");
    return;
  }
  auto const &side = sdf->GetElement("side")->Get<std::string>();
  ph_ = std::make_unique<ros::NodeHandle>(side + "_gripper");

  // Broadcasts state.
  auto const status_topic_name = get<std::string>(sdf, "topic_status", "gripper_status");
  status_pub_ = ph_->advertise<victor_hardware_interface_msgs::Robotiq3FingerStatus>(status_topic_name, 10);

  // Subscribe to user published handle control commands.
  auto const command_topic_name = get<std::string>(sdf, "topic_command", "gripper_command");
  auto gripper_command_sub_options = ros::SubscribeOptions::create<victor_hardware_interface_msgs::Robotiq3FingerCommand>(
      command_topic_name, 1, [this](auto && PH1) { OnGripperCommand(PH1); },
      ros::VoidPtr(), &rosQueue);
  command_sub_ = ph_->subscribe(gripper_command_sub_options);


  // Overload the PID parameters if they are available.
  auto const &kp = get(sdf, "kp_position", 1.0);
  auto const &ki = get(sdf, "kp_position", 0.0);
  auto const &kd = get(sdf, "kp_position", 0.5);
  control_ = std::make_unique<RobotiqControl>(model, kp, ki, kd, side);

  // Start callback queue.
  ros_queue_thread_ = std::thread([this] { QueueThread(); });


  // Connect to gazebo world update.
  updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](gazebo::common::UpdateInfo const &) { control_->UpdateStates(); });

  // Log information.
  ROS_INFO_STREAM("RobotiqHandPlugin loaded for " << side << " hand.");
}

void RobotiqHandPlugin::OnGripperCommand(victor_hardware_interface_msgs::Robotiq3FingerCommandConstPtr const &msg)
{
  robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput cmd;
  cmd.rACT = 1;
  control_->SetHandleCommand(cmd);
}

void RobotiqHandPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (ph_->ok())
  {
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

RobotiqHandPlugin::~RobotiqHandPlugin()
{
  ph_->shutdown();
  rosQueue.clear();
  rosQueue.disable();
  ros_queue_thread_.join();
}

GZ_REGISTER_MODEL_PLUGIN(RobotiqHandPlugin)