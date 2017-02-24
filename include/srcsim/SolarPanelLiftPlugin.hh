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
#ifndef GAZEBO_PLUGINS_SOLARPANELLIFTPLUGIN_HH_
#define GAZEBO_PLUGINS_SOLARPANELLIFTPLUGIN_HH_

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/transport/Node.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE SolarPanelLiftPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: SolarPanelLiftPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for toggle "service".
    /// \param[in] _msg Unused message
    public: void Toggle(ConstIntPtr &/*_msg*/);

    /// \brief Update plugin's function.
    private: void OnUpdate(const common::UpdateInfo &/*_info*/);

    /// \brief Contact sensors attached to large panels
    private: std::vector<sensors::ContactSensorPtr> contactSensors;

    /// \brief Time to lift panel.
    private: common::Time targetTime = 5;

    /// \brief Time when panel started being lifted.
    private: common::Time liftStart;

    /// \brief Gazebo transport node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Publisher which publishes a message once the panel is lifted.
    private: transport::PublisherPtr liftedPub;

    /// \brief Subscriber to toggle messages.
    private: transport::SubscriberPtr toggleSub;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif
