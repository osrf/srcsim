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

#include <gazebo/common/UpdateInfo.hh>

#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/World.hh>

#include "srcsim/BoxContainsPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(BoxContainsPlugin)

/////////////////////////////////////////////////
BoxContainsPlugin::BoxContainsPlugin() : WorldPlugin()
{
}

/////////////////////////////////////////////////
void BoxContainsPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // Load SDF params
  if (!_sdf->HasElement("size"))
  {
    gzerr << "Missing required parameter <size>" << std::endl;
    return;
  }

  if (!_sdf->HasElement("pose"))
  {
    gzerr << "Missing required parameter <pose>" << std::endl;
    return;
  }

  if (!_sdf->HasElement("entity"))
  {
    gzerr << "Missing required parameter <entity>" << std::endl;
    return;
  }

  if (!_sdf->HasElement("namespace"))
  {
    gzerr << "Missing required parameter <namespace>" << std::endl;
    return;
  }

  // Update period
  if (_sdf->HasElement("frequency"))
    this->period = 1.0/_sdf->Get<double>("frequency");
  else
    this->period = 1.0/30.0;

  auto size = _sdf->Get<ignition::math::Vector3d>("size");
  auto pose = _sdf->Get<ignition::math::Pose3d>("pose");
  this->entityName = _sdf->Get<std::string>("entity");
  this->ns = _sdf->Get<std::string>("namespace");

  this->box = ignition::math::OrientedBoxd(size, pose);

  this->world = _world;

  // Start/stop service
  this->ignNode.Advertise("/" + this->ns + "/box/toggle",
      &BoxContainsPlugin::Toggle, this);

  auto start = _sdf->HasElement("start") && _sdf->Get<bool>("start");
  if (start)
  {
    ignition::msgs::StringMsg req;
    bool result;
    this->Toggle(req, result);
  }
}

//////////////////////////////////////////////////
void BoxContainsPlugin::Toggle(ignition::msgs::StringMsg &_rep, bool &_result)
{
  _result = true;

  // Start
  if (!this->updateConnection)
  {
    this->entity = this->world->GetEntity(this->entityName);
    if (!this->entity)
    {
      gzerr << "Can't find entity[" << entity <<
          "] in world. Failed to start Box Plugin." << std::endl;
      _result = false;
    }

    // Start update
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BoxContainsPlugin::OnUpdate, this, std::placeholders::_1));

    this->containsPub = this->ignNode.Advertise<ignition::msgs::Boolean>(
        "/" + this->ns + "/box/contains");

    _rep.set_data("Started");
  }
  // Stop
  else
  {
    this->updateConnection.reset();
    _rep.set_data("Stopped");
  }
}

/////////////////////////////////////////////////
void BoxContainsPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  auto currentTime = _info.simTime;

  // Check for world reset
  if (currentTime < this->prevTime)
    this->prevTime = currentTime;

  // Throttle update
  if ((currentTime - this->prevTime).Double() < this->period)
    return;

  this->prevTime = currentTime;

  // For safety
  if (!this->entity)
  {
    gzerr << "Entity is null" << std::endl;
    return;
  }

  auto pos = this->entity->GetWorldPose().Ign().Pos();
  auto contains = this->box.Contains(pos);

  ignition::msgs::Boolean msg;
  msg.set_data(contains);

  this->containsPub.Publish(msg);
}


