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
#include <srcsim/Harness.h>
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
    gzerr << "Failed to get model pointer."
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

  this->statusRosPub = this->rosNode->advertise<srcsim::Harness>(
      "/srcsim/finals/harness", 1000);
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
  this->latestAnkleForce = _msg->wrench().force().z();
}

//////////////////////////////////////////////////
void HarnessManager::NewGoal(const ignition::math::Pose3d &_pose)
{
  if (!this->model)
    return;

  this->goalChanged = true;
  this->goal = _pose;

  gzmsg << "[Harness] Received new goal [" << _pose << "]" << std::endl;
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
    gzmsg << "[Harness | " <<
        _time.FormattedString(common::Time::MINUTES, common::Time::MILLISECONDS)
        << "] New goal ... ";
    this->TriggerDetach();
    std::cout << "detaching" << std::endl;
    this->transition = DETACH_TO_ATTACH;
    this->goalChanged = false;
  }

  if (this->transition == DETACH_TO_ATTACH && this->IsDetached())
  {
    gzmsg << "[Harness | " <<
        _time.FormattedString(common::Time::MINUTES, common::Time::MILLISECONDS)
        << "] Detached ... ";
    this->TriggerAttach();
    std::cout << "reattaching" << std::endl;
    this->transition = ATTACH_TO_LOWER;
  }
  else if (this->transition == ATTACH_TO_LOWER && this->IsAttached())
  {
    gzmsg << "[Harness | " <<
        _time.FormattedString(common::Time::MINUTES, common::Time::MILLISECONDS)
        << "] Attached ... "
        << " resetting joint positions ... ";
    this->ResetJointPositions();
    this->TriggerLower();
    std::cout << "lowering" << std::endl;
    this->transition = LOWER_TO_STAND;
    this->itLowering = 0;
  }
  else if (this->transition == LOWER_TO_STAND && this->IsLowered())
  {
    gzmsg << "[Harness | " <<
        _time.FormattedString(common::Time::MINUTES, common::Time::MILLISECONDS)
        << "] Lowered ... ";
    this->TriggerStand();
    std::cout << "switching to high level control" << std::endl;
    this->transition = STAND_TO_DETACH;
    this->itStanding = 0;
  }
  else if (this->transition == STAND_TO_DETACH && this->IsStanding())
  {
    gzmsg << "[Harness | " <<
        _time.FormattedString(common::Time::MINUTES, common::Time::MILLISECONDS)
        << "] Standing up ... ";
    this->TriggerDetach();
    std::cout << "detaching" << std::endl;
    this->transition = DETACH_TO_NONE;
  }
  else if (this->transition == DETACH_TO_NONE && this->IsDetached())
  {
    gzmsg << "[Harness | " <<
        _time.FormattedString(common::Time::MINUTES, common::Time::MILLISECONDS)
        << "] Detached!" << std::endl;
    this->transition = NONE;
  }

  srcsim::Harness msg;
  msg.status = this->transition;
  this->statusRosPub.publish(msg);
}

//////////////////////////////////////////////////
void HarnessManager::ResetJointPositions()
{
  std::string prefix = this->model->GetScopedName() + "::";
  std::map<std::string, double> jps;
  jps.insert(std::make_pair(prefix + "leftHipYaw", -0.01));
  jps.insert(std::make_pair(prefix + "leftHipRoll", -0.06));
  jps.insert(std::make_pair(prefix + "leftHipPitch", -0.51));
  jps.insert(std::make_pair(prefix + "leftKneePitch", 1.13));
  jps.insert(std::make_pair(prefix + "leftAnklePitch", -0.61));
  jps.insert(std::make_pair(prefix + "leftAnkleRoll", 0.0));
  jps.insert(std::make_pair(prefix + "rightHipYaw", 0.01));
  jps.insert(std::make_pair(prefix + "rightHipRoll", 0.05));
  jps.insert(std::make_pair(prefix + "rightHipPitch", -0.51));
  jps.insert(std::make_pair(prefix + "rightKneePitch", 1.13));
  jps.insert(std::make_pair(prefix + "rightAnklePitch", -0.61));
  jps.insert(std::make_pair(prefix + "rightAnkleRoll", 0.0));
  // The pelvis is the root of the urdf,
  // but the harness is attached to the torso
  // Due to a limitation of Joint::SetPosition,
  // it won't work properly to set the joints in between torso and pelvis
  // but the legs should swing down and settle properly
  // jps.insert(std::make_pair(prefix + "torsoYaw", 0.0));
  // jps.insert(std::make_pair(prefix + "torsoPitch", 0.0));
  // jps.insert(std::make_pair(prefix + "torsoRoll", 0.0));
  jps.insert(std::make_pair(prefix + "leftShoulderPitch", 0.07));
  jps.insert(std::make_pair(prefix + "leftShoulderRoll", -1.45));
  jps.insert(std::make_pair(prefix + "leftShoulderYaw", 0.07));
  jps.insert(std::make_pair(prefix + "leftElbowPitch", -1.03));
  jps.insert(std::make_pair(prefix + "leftForearmYaw", 0.39));
  jps.insert(std::make_pair(prefix + "leftWristRoll", 0.0));
  jps.insert(std::make_pair(prefix + "leftWristPitch", 0.0));
  jps.insert(std::make_pair(prefix + "lowerNeckPitch", 0.0));
  jps.insert(std::make_pair(prefix + "neckYaw", 0.0));
  jps.insert(std::make_pair(prefix + "upperNeckPitch", 0.0));
  jps.insert(std::make_pair(prefix + "hokuyo_joint", 0.0));
  jps.insert(std::make_pair(prefix + "rightShoulderPitch", 0.10));
  jps.insert(std::make_pair(prefix + "rightShoulderRoll", 1.42));
  jps.insert(std::make_pair(prefix + "rightShoulderYaw", 0.08));
  jps.insert(std::make_pair(prefix + "rightElbowPitch", 1.06));
  jps.insert(std::make_pair(prefix + "rightForearmYaw", 0.39));
  jps.insert(std::make_pair(prefix + "rightWristRoll", 0.0));
  jps.insert(std::make_pair(prefix + "rightWristPitch", 0.0));
  model->SetJointPositions(jps);
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
bool HarnessManager::IsDetached()
{
  return this->model->GetJoint("harness_joint") == nullptr;
}

//////////////////////////////////////////////////
bool HarnessManager::IsAttached()
{
  return !this->IsDetached();
}

//////////////////////////////////////////////////
bool HarnessManager::IsLowered()
{
  this->itLowering = this->latestAnkleForce > 50 ? this->itLowering + 1 : 0;
  return this->itLowering >= this->itThreshold;
}

//////////////////////////////////////////////////
bool HarnessManager::IsStanding()
{
  this->itStanding = this->latestAnkleForce > 500 ? this->itStanding + 1 : 0;
  return this->itStanding >= this->itThreshold;
}

