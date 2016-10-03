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

#include <cstdlib>
#include <functional>
#include <boost/filesystem.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/util/util.hh>
#include <sdf/sdf.hh>

#include "srcsim/LogPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(LogPlugin)

/////////////////////////////////////////////////
LogPlugin::LogPlugin()
{
}

/////////////////////////////////////////////////
void LogPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;

  this->CreateLogFile(_sdf);

  // Open the log file for writing.
  this->logFileStream.open(this->logFilePath.string().c_str(),
      std::fstream::out);
  if (!this->logFileStream.is_open())
  {
    gzerr << "Failed to open log file :" << this->logFilePath << std::endl;
    return;
  }
  gzlog << "Writing log data to " << this->logFilePath << std::endl;

  // Listen to the update event. This event is broadcast every sim iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LogPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void LogPlugin::CreateLogFile(const sdf::ElementPtr _sdf)
{
  // Set the log filename.
  if (_sdf->HasElement("log_file"))
  {
    this->logFilePath =
      boost::filesystem::path(_sdf->Get<std::string>("log_file"));
  }
  else
  {
    // Get the user's home directory.
    char *homePath = std::getenv("HOME");

    if (!homePath)
      this->logFilePath = boost::filesystem::path("/tmp/SRC_gazebo");
    else
      this->logFilePath = boost::filesystem::path(homePath);

    this->logFilePath /= "logs";
    this->logFilePath /= this->world->GetName() + "log";
  }

  // Create the log directory if needed.
  if (!boost::filesystem::exists(this->logFilePath.parent_path()))
    boost::filesystem::create_directories(this->logFilePath.parent_path());
}

/////////////////////////////////////////////////
void LogPlugin::OnUpdate()
{

}

/////////////////////////////////////////////////
void LogPlugin::WriteLog(const common::Time &_simTime,
  const common::Time &_wallTime, const std::string &_msg, const bool _force)
{
  // Write at 1Hz.
  if (!_force && (_simTime - this->prevLogTime).Double() < 1.0)
    return;

  // If we're being forced, that means that something interesting happened.
  // Also force the gazebo state logger to write.
  if (_force)
  {
    gzdbg << "LogPlugin forcing LogRecord to write" << std::endl;
    util::LogRecord::Instance()->Notify();
  }

  if (!this->logFileStream.is_open())
  {
    gzerr << "Log file stream is no longer open:" << this->logFilePath <<
        std::endl;
    return;
  }
}
