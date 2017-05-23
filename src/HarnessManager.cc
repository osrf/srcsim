/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <ihmc_valkyrie_ros/ValkyrieLowLevelControlModeRosMessage.h>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include "srcsim/HarnessManager.hh"

using namespace gazebo;

//////////////////////////////////////////////////
HarnessManager::HarnessManager()
{
  auto world = physics::get_world();
  if (!world)
  {
    gzerr << "Failed to get world pointer."
        << std::endl;
    return;
  }

  this->model = world->GetModel("valkyrie");
  if (!this->model)
  {
    gzerr << "Failed to get model pointe."
        << std::endl;
    return;
  }

  // Gazebo transport
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();

  this->detachGzPub = this->gzNode->Advertise<msgs::GzString>(
      "~/valkyrie/harness/detach");
  this->attachGzPub = this->gzNode->Advertise<msgs::Pose>(
      "~/valkyrie/harness/attach");
  this->lowerGzPub = this->gzNode->Advertise<msgs::GzString>(
      "~/valkyrie/harness/velocity");
  this->sensorGzSub = this->gzNode->Subscribe(
      "~/valkyrie/rightAnkleRoll/rightFootSixAxis/wrench",
      &HarnessManager::OnSensorMsg, this);

  // ROS transport
  this->rosNode.reset(new ros::NodeHandle());

  this->controlRosPub = this->rosNode->advertise<
      ihmc_valkyrie_ros::ValkyrieLowLevelControlModeRosMessage>(
      "/ihmc_ros/valkyrie/control/low_level_control_mode", 1000);
}

//////////////////////////////////////////////////
HarnessManager::~HarnessManager()
{
  this->detachGzPub.reset();
  this->attachGzPub.reset();
  this->lowerGzPub.reset();
  this->gzNode->Fini();
  this->gzNode.reset();
}

//////////////////////////////////////////////////
void HarnessManager::OnSensorMsg(ConstWrenchStampedPtr &_msg)
{
  this->lastSensor = _msg->wrench().force().z();
}

//////////////////////////////////////////////////
void HarnessManager::NewGoal(const ignition::math::Pose3d &_pose)
{
  if (!this->model)
    return;

  this->goalChanged = true;
  this->goal = _pose;
}

//////////////////////////////////////////////////
void HarnessManager::Update(const common::Time &_time)
{
  // Do nothing unless we got a new goal
  if (this->transition == NONE && !this->goalChanged)
    return;

  // No matter what we're doing, if a new goal is received, detach first
  if (this->goalChanged)
  {
    gzmsg << "[Harness] New goal ... ";
    this->TriggerDetach();
    std::cout << "detaching" << std::endl;
    this->transition = DETACH_TO_ATTACH;
    this->goalChanged = false;
  }

  if (this->transition == DETACH_TO_ATTACH && !this->IsAttached())
  {
    gzmsg << "[Harness] Detached ... ";
    this->TriggerAttach();
    std::cout << "reattaching" << std::endl;
    this->transition = ATTACH_TO_LOWER;
  }
  else if (this->transition == ATTACH_TO_LOWER && this->IsAttached())
  {
    gzmsg << "[Harness] Attached ... ";
    this->TriggerLower();
    std::cout << "lowering" << std::endl;
    this->transition = LOWER_TO_STAND;
  }
  else if (this->transition == LOWER_TO_STAND && this->IsLowered())
  {
    gzmsg << "[Harness] Lowered ... ";
    this->TriggerStand();
    std::cout << "switching to high level control" << std::endl;
    this->transition = STAND_TO_DETACH;
  }
  else if (this->transition == STAND_TO_DETACH && this->IsStanding())
  {
    gzmsg << "[Harness] Standing up ... ";
    this->TriggerDetach();
    std::cout << "detaching" << std::endl;
    this->transition = DETACH_TO_NONE;
  }
  else if (this->transition == DETACH_TO_NONE && !this->IsAttached())
  {
    gzmsg << "[Harness] Detached" << std::endl;
    this->transition = NONE;
  }
}

//////////////////////////////////////////////////
void HarnessManager::TriggerDetach()
{
  if (!this->IsAttached())
    return;

  msgs::GzString msg;
  msg.set_data("true");
  this->detachGzPub->Publish(msg);
}

//////////////////////////////////////////////////
void HarnessManager::TriggerAttach()
{
  msgs::Pose msg;
  msgs::Set(&msg, this->goal);
  this->attachGzPub->Publish(msg);
}

//////////////////////////////////////////////////
void HarnessManager::TriggerLower()
{
  msgs::GzString msg;
  msg.set_data("-0.05");
  this->lowerGzPub->Publish(msg);
}

//////////////////////////////////////////////////
void HarnessManager::TriggerStand()
{
  ihmc_valkyrie_ros::ValkyrieLowLevelControlModeRosMessage msg;
  msg.requested_control_mode = 2;
  msg.unique_id = -1;
  this->controlRosPub.publish(msg);
}

//////////////////////////////////////////////////
bool HarnessManager::IsAttached()
{
  return this->model->GetJoint("harness_joint") != nullptr;
}

//////////////////////////////////////////////////
bool HarnessManager::IsLowered()
{
  // TODO: check for a few iterations
  return this->lastSensor > 50;
}

//////////////////////////////////////////////////
bool HarnessManager::IsStanding()
{
  return this->lastSensor > 250;
}

