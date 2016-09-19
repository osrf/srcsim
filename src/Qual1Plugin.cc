/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "srcsim/Qual1Plugin.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/transport/Publisher.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(Qual1Plugin)

/////////////////////////////////////////////////
Qual1Plugin::Qual1Plugin()
{
}

/////////////////////////////////////////////////
void Qual1Plugin::Load(physics::WorldPtr /*_world*/, sdf::ElementPtr /*_sdf*/)
{
  this->node.reset(new gazebo::transport::Node());
  this->node->Init();
  this->pub = node->Advertise<gazebo::msgs::Visual>("~/visual");

  this->requestPub =
    this->node->Advertise<gazebo::msgs::Request>("~/request");

  this->responseSub = this->node->Subscribe("~/response",
      &Qual1Plugin::OnResponse, this, true);

  this->ignNode.Subscribe("/light", &Qual1Plugin::OnLight, this);

  // Wait for a subscriber to connect
  this->pub->WaitForConnection();

  this->requestPub->WaitForConnection();
  this->requestMsg = gazebo::msgs::CreateRequest("scene_info");
  this->requestPub->Publish(*this->requestMsg);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&Qual1Plugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void Qual1Plugin::Switch(int _console, int _light,
                         const gazebo::common::Color &_clr)
{
  std::ostringstream name, parent;
  name << "console" << _console << "::visuals::light" << _light;
  parent << "console" << _console << "::visuals";

  gazebo::msgs::Visual msg;
  msg.set_name(name.str());
  msg.set_parent_name(parent.str());
  gazebo::msgs::Set(msg.mutable_material()->mutable_ambient(), _clr);
  gazebo::msgs::Set(msg.mutable_material()->mutable_diffuse(), _clr);
  gazebo::msgs::Set(msg.mutable_material()->mutable_emissive(), _clr);

  this->activeConsole = _console;
  this->activeLight = _light;
  // std::cout << "Switch[" << msg.DebugString() << "]\n";

  this->pub->Publish(msg);
}

/////////////////////////////////////////////////
void Qual1Plugin::OnLight(const ignition::msgs::Vector3d &/*_msg*/)
{
}

/////////////////////////////////////////////////
void Qual1Plugin::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    return;

  gazebo::msgs::Scene sceneMsg;
  sceneMsg.ParseFromString(_msg->serialized_data());

  // The following set of nested loops finds all the console
  // lights that can be switched on/off.
  for (int i = 0; i < sceneMsg.model_size(); ++i)
  {
    // Find the console models
    if (sceneMsg.model(i).name().find("console") == 0)
    {
      std::string consoleName = sceneMsg.model(i).name();
      int consoleNum = std::stoi(consoleName.substr(7));

      // Loop through all links
      for (int j = 0; j < sceneMsg.model(i).link_size(); ++j)
      {
        // Find the link that has the light visuals
        if (sceneMsg.model(i).link(j).name() ==
            consoleName + "::visuals")
        {
          // Process each visual
          for (int k = 0;
              k < sceneMsg.model(i).link(j).visual_size(); ++k)
          {
            if (sceneMsg.model(i).link(j).visual(k).name().find(
                  consoleName + "::visuals::light") == 0)
            {
              int lightNum = std::stoi(
                  sceneMsg.model(i).link(j).visual(k).name().substr(
                    24));

              if (this->lights.find(consoleNum) == this->lights.end()
                  || this->lights[consoleNum].find(lightNum) ==
                  this->lights[consoleNum].end())
              {

                std::cout << "Console[" << consoleNum << "] LightNum[" << lightNum << "]\n";
                this->lights[consoleNum][lightNum] =
                  gazebo::msgs::ConvertIgn(
                      sceneMsg.model(i).link(j).visual(k).pose());
              }
            }
          }
        }
      }
    }
  }

  this->requestMsg = nullptr;
}

/////////////////////////////////////////////////
void Qual1Plugin::OnUpdate()
{
  this->Switch(1, 2, gazebo::common::Color::Red);
}
