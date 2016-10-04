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

#include <fstream>
#include <string>
#include <boost/filesystem.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief Plugin that logs relevant information for scoring and
  /// task postprocessing.
  class GAZEBO_VISIBLE LogPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: LogPlugin();

    /// \brief Destructor.
    public: virtual ~LogPlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to the world.
    /// \param[in] _sdf Pointer to the plugin's sdf element.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Create an empty log file in the appropriate path.
    /// \param[in] _sdf Pointer to the plugin's sdf element.
    private: void CreateLogFile(const sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    /// \param[in] _info Current world information.
    private: virtual void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Write intermediate log data.
    /// \param _simTime Current simulation time.
    /// \param _wallTime Current wallclock time.
    /// \param _msg Log message to include.
    private: void WriteLog(const gazebo::common::Time& _simTime,
      const common::Time &_wallTime, const std::string &_msg);

    /// \brief Create a string with the specific data to log during qual #1.
    /// \return A string representation of the specific data to log.
    private: std::string LogQual1() const;

    /// \brief Create a string with the specific data to log during qual #2.
    /// \return A string representation of the specific data to log.
    private: std::string LogQual2() const;

    /// \brief Create a string with the specific data to log during final #1.
    /// \return A string representation of the specific data to log.
    private: std::string LogFinal1() const;

    /// \brief Create a string with the specific data to log during final #2.
    /// \return A string representation of the specific data to log.
    private: std::string LogFinal2() const;

    /// \brief Create a string with the specific data to log during final #3.
    /// \return A string representation of the specific data to log.
    private: std::string LogFinal3() const;

    /// \brief The worlds that we might be logging; each one can be
    /// slightly different.
    private: enum WorldType
             {
               QUAL_1,
               QUAL_2,
               FINAL_1,
               FINAL_2,
               FINAL_3
             };

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the world.
    private: gazebo::physics::WorldPtr world;

    /// \brief Name of the file that we're writing log data to.
    private: boost::filesystem::path logFilePath;

    /// \brief The stream associated with logFilePath.
    private: std::ofstream logFileStream;

    /// \brief Which type of world we're scoring
    private: enum WorldType worldType;
  };
}

#endif
