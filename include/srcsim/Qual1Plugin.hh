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
#ifndef GAZEBO_PLUGINS_WINDPLUGIN_HH_
#define GAZEBO_PLUGINS_WINDPLUGIN_HH_

#include <ignition/transport.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE Qual1Plugin : public WorldPlugin
  {
    /// \brief Constructor
    public: Qual1Plugin();

    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    private: void OnResponse(ConstResponsePtr &_msg);

    private: void Switch(int _console, int _light,
                         const gazebo::common::Color &_clr);

    private: void OnLight(const ignition::msgs::Vector3d &_msg);

    /// \brief Callback for World Update events.
    private: virtual void OnUpdate();

    private: ignition::transport::Node ignNode;
    private: gazebo::transport::NodePtr node;
    private: gazebo::transport::PublisherPtr pub;
    private: gazebo::transport::PublisherPtr requestPub;
    private: gazebo::transport::SubscriberPtr responseSub;
    private: gazebo::msgs::Request *requestMsg = nullptr;

    private: int activeConsole = 0;
    private: int activeLight = 0;

    private: std::map<int, std::map<int, ignition::math::Pose3d>> lights;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif
