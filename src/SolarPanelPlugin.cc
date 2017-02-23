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

#include <functional>
#include <string>
#include <gazebo/common/Assert.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>

#include "srcsim/SolarPanelPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SolarPanelPlugin)

/////////////////////////////////////////////////
SolarPanelPlugin::SolarPanelPlugin()
{
}

/////////////////////////////////////////////////
void SolarPanelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");
  GZ_ASSERT(_sdf, "SDF pointer is null");

  this->joints.push_back(_model->GetJoint("base_panel_01"));
  this->joints.push_back(_model->GetJoint("base_panel_02"));
  this->joints.push_back(_model->GetJoint("panel_01_panel_small_01"));
  this->joints.push_back(_model->GetJoint("panel_01_panel_small_03"));
  this->joints.push_back(_model->GetJoint("panel_02_panel_small_02"));
  this->joints.push_back(_model->GetJoint("panel_02_panel_small_04"));

  for (const auto &it : this->joints)
  {
    if (!it)
    {
      gzerr << "Some joint was not found" << std::endl;
      return;
    }
  }

  this->buttonJoint = _model->GetJoint("button");
  if (!this->buttonJoint)
  {
    gzerr << "Joint [button] not found" << std::endl;
    return;
  }

  this->lowerLimit = this->buttonJoint->GetLowerLimit(0).Radian();
  this->upperLimit = this->buttonJoint->GetUpperLimit(0).Radian();
  this->range = this->upperLimit - this->lowerLimit;

  // Start/stop "service"
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->toggleSub = this->gzNode->Subscribe("/task2/checkpoint3/toggle",
      &SolarPanelPlugin::Toggle, this);

  // Start enabled or not
  auto enabled = _sdf->HasElement("enabled") && _sdf->Get<bool>("enabled");
  if (enabled)
  {
    ConstIntPtr msg;
    this->Toggle(msg);
  }
}

//////////////////////////////////////////////////
void SolarPanelPlugin::Toggle(ConstIntPtr &/*_msg*/)
{
  // Start
  if (!this->updateConnection)
  {
    // Start update
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SolarPanelPlugin::OnUpdate, this, std::placeholders::_1));

    this->openedPub = this->gzNode->Advertise<msgs::Int>(
        "/task2/checkpoint3/opened");
    gzmsg << "Started solar panel plugin" << std::endl;
  }
  // Stop
  else
  {
    this->updateConnection.reset();
    gzmsg << "Stopped solar panel plugin" << std::endl;
  }
}

/////////////////////////////////////////////////
void SolarPanelPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  // Something went wrong, nothing to do.
  if (!this->buttonJoint || !(this->joints.size() == 6u))
    return;

  // It's enough to press the button once
  if (!this->pressed)
  {
    int percentagePressed = 100 -
      ((this->buttonJoint->GetAngle(0).Radian() - this->lowerLimit) /
        this->range) * 100;

    this->pressed = percentagePressed >= this->kPercentageButtonPressed;
  }

  // Nothing to do yet
  if (!this->pressed)
    return;

  auto largeForce = 1.0;
  auto smallForce = 0.5;

  bool p1open = this->joints[0]->GetAngle(0).Radian() <= -IGN_PI * 0.4;
  bool p2open = this->joints[1]->GetAngle(0).Radian() >=  IGN_PI * 0.4;
  bool ps1open = this->joints[2]->GetAngle(0).Radian() >=  IGN_PI * 0.8;
  bool ps2open = this->joints[3]->GetAngle(0).Radian() <= -IGN_PI * 0.8;
  bool ps3open = this->joints[4]->GetAngle(0).Radian() <= -IGN_PI * 0.8;
  bool ps4open = this->joints[5]->GetAngle(0).Radian() >=  IGN_PI * 0.8;

  // Large panel 1
  if (!p1open)
  {
    this->joints[0]->SetForce(0, -largeForce);
  }

  // Large panel 2
  if (!p2open)
  {
    this->joints[1]->SetForce(0, largeForce);
  }

  // Small panel 1
  if (p1open && p2open && !ps1open)
  {
    this->joints[2]->SetForce(0, smallForce);
  }

  // Small panel 2
  if (p1open && p2open && !ps2open)
  {
    this->joints[3]->SetForce(0, -smallForce);
  }

  // Small panel 3
  if (p1open && p2open && !ps3open)
  {
    this->joints[4]->SetForce(0, -smallForce);
  }

  // Small panel 4
  if (p1open && p2open && !ps4open)
  {
    this->joints[5]->SetForce(0, smallForce);
  }

  // This is a single-use plugin. After all panels are open, publish a message
  // and stop updating
  if (p1open && p2open && ps1open && ps2open && ps3open && ps4open)
  {
    gazebo::msgs::Int msg;
    msg.set_data(1);

    this->openedPub->Publish(msg);

    this->updateConnection.reset();

    gzmsg << "Solar panel is open" << std::endl;
  }
}
