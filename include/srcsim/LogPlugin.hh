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
#ifndef GAZEBO_PLUGINS_LOGPLUGIN_HH_
#define GAZEBO_PLUGINS_LOGPLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \brief Plugin that logs relevant information for scoring and
  /// postprocessing.
  class GAZEBO_VISIBLE LogPlugin : public WorldPlugin
  {
    /// \brief Constructor
    public: LogPlugin();

    /// \brief Load the plugin
    /// \param[in] _world Pointer to the world.
    /// \param[in] _sdf Pointer to the plugin's sdf element.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: virtual void OnUpdate();

    /// \brief Gazebo transport node
    private: gazebo::transport::NodePtr node;

    /// \brief Gazebo publisher for updating visuals.
    private: gazebo::transport::PublisherPtr pub;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the world.
    private: gazebo::physics::WorldPtr world;

    /// \brief Mutex used to prevent interleaved messages.
    private: std::mutex mutex;
  };
}

#endif
