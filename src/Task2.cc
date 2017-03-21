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

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorManager.hh>

#include "srcsim/Task2.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Task2::Task2(const common::Time &_timeout,
    const std::vector<ignition::math::Pose3d> _poses)
    : Task(_timeout)
{
  // Checkpoint 3: Deploy solar panel
  std::unique_ptr<Task2CP3> cp3(new Task2CP3(_poses[2]));
  this->checkpoints.push_back(std::move(cp3));

  // Checkpoint 5: Plug cable
  std::unique_ptr<Task2CP5> cp5(new Task2CP5(_poses[4]));
  this->checkpoints.push_back(std::move(cp5));

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
bool Task2CP5::Check()
{
  // Previously finished
  if (this->done)
    return true;

  // First time
  if (!this->sensor)
  {
    this->world = physics::get_world();

    if (this->world)
    {
      this->sensor = std::dynamic_pointer_cast<sensors::ContactSensor>(
          sensors::SensorManager::Instance()->GetSensor(this->sensorName));
      this->sensor->SetActive(true);
    }
  }

  // If couldn't get sensor
  if (!this->sensor)
  {
    gzerr << "Couldn't get world or sensor pointer. "
          << "Not checking Task 2 Checkpoint 5." << std::endl;
    return false;
  }

  // Get all contacts
  auto contacts = this->sensor->Contacts();

  bool touching = false;

  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // Check for the plug
    bool col1Plug = contacts.contact(i).collision1().find(this->plug) !=
        std::string::npos;
    bool col2Plug = contacts.contact(i).collision2().find(this->plug) !=
        std::string::npos;

    // Check for the outlet
    bool col1Outlet = contacts.contact(i).collision1().find(this->outlet) !=
        std::string::npos;
    bool col2Outlet = contacts.contact(i).collision2().find(this->outlet) !=
        std::string::npos;

    // We need at least one plug-outlet or outlet-plug contact
    if ((col1Plug && col2Outlet) || (col1Outlet && col2Plug))
    {
      touching = true;
      break;
    }
  }

  auto simTime = world->GetSimTime();

  // Not touching
  if (!touching)
  {
    // Stopped touching
    if (this->touchStart != common::Time::Zero)
    {
      gzmsg << "Plug stopped touching outlet at " << simTime
            << " seconds" << std::endl;
      this->touchStart = common::Time::Zero;
    }
    return false;
  }

  // Just started touching
  if (touching && this->touchStart == common::Time::Zero)
  {
    this->touchStart = simTime;
    gzmsg << "Plug started touching outlet at " << this->touchStart
          << " seconds" << std::endl;
  }

  // Check if it has been plugged for long enough
  auto itsPlugTime = simTime - this->touchStart > this->targetTime;

  if (!itsPlugTime)
    return false;

  // We set done to true because the checkpoint has technically been completed.
  // If we fail to create a joint below, this is not the competitor's fault.
  this->done = true;

  // Create fixed joint

  // Get models
  auto cableModel = world->GetModel(this->cable);
  auto plugLink = boost::dynamic_pointer_cast<physics::Link>(
      world->GetEntity(this->plug));

  auto outletLink = boost::dynamic_pointer_cast<physics::Link>(
      world->GetEntity(this->outletParent));

  if (!cableModel || !plugLink || !outletLink)
  {
    gzerr << "Failed to get a model, cable won't be plugged."
          << std::endl;
    return this->done;
  }

  auto fixedJoint = world->GetPhysicsEngine()->CreateJoint("fixed", cableModel);
  fixedJoint->SetName(this->cable + "_plug_fixed_joint__");

  fixedJoint->Load(plugLink, outletLink, ignition::math::Pose3d::Zero);
  fixedJoint->Init();

  gzmsg << "The cable plug has been fixed to the outlet" << std::endl;

  return this->done;
}

/////////////////////////////////////////////////
bool Task2CP6::Check()
{
  return this->CheckBox("/task2/checkpoint6");
}

