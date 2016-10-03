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

#include <mutex>
#include <boost/filesystem.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>

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

    /// \brief Create an empty log file in the appropriate path.
    /// \param[in] _sdf Pointer to the plugin's sdf element.
    private: void CreateLogFile(const sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: virtual void OnUpdate();

    /// \brief Write intermediate log data
    /// \param _simTime Current simulation time
    /// \param _wallTime Current wallclock time
    /// \param _msg Log message to include
    /// \param _force If true, write output; otherwise write output only if
    /// enough time has passed since the last write.
    private: void WriteLog(const gazebo::common::Time& _simTime,
      const common::Time &_wallTime, const std::string &_msg,
      const bool _force);

    /// \brief Gazebo transport node
    private: gazebo::transport::NodePtr node;

    /// \brief Gazebo publisher for updating visuals.
    private: gazebo::transport::PublisherPtr pub;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the world.
    private: gazebo::physics::WorldPtr world;

    /// \brief Name of the file that we're writing log data to.
    private: boost::filesystem::path logFilePath;

    /// \brief The stream associated with logFilePath.
    private: std::ofstream logFileStream;

    /// \brief When we last wrote log data to disk.
    private: common::Time prevLogTime;

    /// \brief Mutex used to prevent interleaved messages.
    private: std::mutex mutex;
  };
}

#endif
