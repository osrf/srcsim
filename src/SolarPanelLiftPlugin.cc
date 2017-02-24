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
#include <gazebo/sensors/SensorManager.hh>
#include <sdf/sdf.hh>

#include "srcsim/SolarPanelLiftPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SolarPanelLiftPlugin)

/////////////////////////////////////////////////
SolarPanelLiftPlugin::SolarPanelLiftPlugin()
{
}

/////////////////////////////////////////////////
void SolarPanelLiftPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");
  GZ_ASSERT(_sdf, "SDF pointer is null");

  // Contact sensors
  this->contactSensors.push_back(
      std::dynamic_pointer_cast<sensors::ContactSensor>(
      sensors::SensorManager::Instance()->GetSensor("panel_01_contact")));
  this->contactSensors.push_back(
      std::dynamic_pointer_cast<sensors::ContactSensor>(
      sensors::SensorManager::Instance()->GetSensor("panel_02_contact")));
  this->contactSensors.push_back(
      std::dynamic_pointer_cast<sensors::ContactSensor>(
      sensors::SensorManager::Instance()->GetSensor("base_contact")));
  if (!this->contactSensors[0] || !this->contactSensors[1] ||
      !contactSensors[2])
  {
    gzerr << "Some contact sensor not found" << std::endl;
    return;
  }

  // Start/stop "service"
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->toggleSub = this->gzNode->Subscribe("/task2/checkpoint1/toggle",
      &SolarPanelLiftPlugin::Toggle, this);

  // Start enabled or not
  auto enabled = _sdf->HasElement("enabled") && _sdf->Get<bool>("enabled");
  if (enabled)
  {
    ConstIntPtr msg;
    this->Toggle(msg);
  }
}

//////////////////////////////////////////////////
void SolarPanelLiftPlugin::Toggle(ConstIntPtr &/*_msg*/)
{
  // Start
  if (!this->liftedPub)
  {
    // Start update
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SolarPanelLiftPlugin::OnUpdate, this, std::placeholders::_1));

    this->liftedPub = this->gzNode->Advertise<msgs::Int>(
        "/task2/checkpoint1/lifted");

    this->contactSensors[0]->SetActive(true);
    this->contactSensors[1]->SetActive(true);
    this->contactSensors[2]->SetActive(true);

    gzmsg << "Started solar panel lift plugin" << std::endl;
  }
  // Stop
  else
  {
    this->updateConnection.reset();
    this->contactSensors[0]->SetActive(false);
    this->contactSensors[1]->SetActive(false);
    this->contactSensors[2]->SetActive(false);

    gzmsg << "Stopped solar panel lift plugin" << std::endl;
  }
}

/////////////////////////////////////////////////
void SolarPanelLiftPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Get all observed contacts
  auto contacts = this->contactSensors[0]->Contacts();
  contacts.MergeFrom(this->contactSensors[1]->Contacts());
  contacts.MergeFrom(this->contactSensors[2]->Contacts());

  bool touchingVal = false;

  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // Check for the robot
    bool col1Val = contacts.contact(i).collision1().find("valkyrie") !=
        std::string::npos;
    bool col2Val = contacts.contact(i).collision2().find("valkyrie") !=
        std::string::npos;

    if (col1Val || col2Val)
      touchingVal = true;

    // The other collision is the panel
    bool col1Panel = contacts.contact(i).collision1().find("solar_panel") !=
        std::string::npos;
    bool col2Panel = contacts.contact(i).collision2().find("solar_panel") !=
        std::string::npos;

    // If the collisions are not val-panel or panel-val, we're touching
    // something else
    if (!((col1Val && col2Panel) || (col1Panel && col2Val)))
    {
      this->liftStart = common::Time::Zero;
      return;
    }
  }

  // Free-fall is not holding
  if (!touchingVal)
  {
    // Sanity check
    if (contacts.contact_size() > 0)
    {
      gzerr << "Not touching Val, but touching something? "
            << "We shouldn't reach this point" << std::endl;
    }

    this->liftStart = common::Time::Zero;
    return;
  }

  // Check if it has been lifted for long enough
  if (this->liftStart == common::Time::Zero)
  {
    this->liftStart = _info.simTime;
    gzdbg << "Panel lift start [" << this->liftStart << "]" << std::endl;
  }

  auto completed = _info.simTime - this->liftStart > this->targetTime;

  // This is a single-use plugin. After lifted, publish a message
  // and stop updating
  if (completed)
  {
    gzmsg << "Solar panel lifted" << std::endl;

    gazebo::msgs::Int msg;
    msg.set_data(1);

    this->liftedPub->Publish(msg);

    this->updateConnection.reset();
  }
}
