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

#include "srcsim/Task2.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Task2::Task2(const common::Time &_timeout,
    const std::vector<ignition::math::Pose3d> _poses)
    : Task(_timeout)
{
  // Checkpoint 1: Lift solar panel
  std::unique_ptr<Task2CP1> cp1(new Task2CP1(_poses[0]));
  this->checkpoints.push_back(std::move(cp1));

  // Checkpoint 2: Place solar panel near cable
  std::unique_ptr<Task2CP2> cp2(new Task2CP2(_poses[1]));
  this->checkpoints.push_back(std::move(cp2));

  // Checkpoint 3: Deploy solar panel
  std::unique_ptr<Task2CP3> cp3(new Task2CP3(_poses[2]));
  this->checkpoints.push_back(std::move(cp3));

  // Checkpoint 6: Walk to final box
  std::unique_ptr<Task2CP6> cp6(new Task2CP6(_poses[5]));
  this->checkpoints.push_back(std::move(cp6));

  gzmsg << "Task [2] created" << std::endl;
}

/////////////////////////////////////////////////
size_t Task2::Number() const
{
  return 2u;
}

/////////////////////////////////////////////////
bool Task2CP1::Check()
{
  return this->CheckTouch("/task2/checkpoint1");
}

/////////////////////////////////////////////////
bool Task2CP2::Check()
{
  return this->CheckBox("/task2/checkpoint2");
}

/////////////////////////////////////////////////
void Task2CP3::OnSolarPanelGzMsg(ConstIntPtr &/*_msg*/)
{
  this->panelDone = true;
}

/////////////////////////////////////////////////
bool Task2CP3::Check()
{
  // First time
  if (!this->panelGzSub && !this->panelDone)
  {
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Enable solar panel plugin
    this->toggleGzPub = this->gzNode->Advertise<msgs::Int>(
        "/task2/checkpoint3/toggle");

    msgs::Int msg;
    msg.set_data(1);
    this->toggleGzPub->Publish(msg);

    // Subscribe to solar panel msgs
    this->panelGzSub = this->gzNode->Subscribe("/task2/checkpoint3/opened",
        &Task2CP3::OnSolarPanelGzMsg, this);
  }

  if (this->panelDone)
  {
    msgs::Int msg;
    msg.set_data(0);
    this->toggleGzPub->Publish(msg);

    this->panelGzSub.reset();
  }

  return this->panelDone;
}

/////////////////////////////////////////////////
bool Task2CP6::Check()
{
  return this->CheckBox("/task2/checkpoint6");
}

