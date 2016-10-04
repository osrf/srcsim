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
#include <sstream>
#include <boost/filesystem.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Model.hh>
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
LogPlugin::~LogPlugin()
{
  if (this->logFileStream.is_open())
    this->logFileStream.close();
}


/////////////////////////////////////////////////
void LogPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;

  gzlog << "SRC LogPlugin: world name is [" <<
    this->world->GetName() << "]" << std::endl;
  if (this->world->GetName() == "SRC_qual1")
    this->worldType = QUAL_1;
  else if (this->world->GetName() == "SRC_qual2")
    this->worldType = QUAL_2;
  else
  {
    gzerr << "SRC LogPlugin: unknown world name [" <<
      this->world->GetName() << "]; no custom logging available.";
    return;
  }

  this->CreateLogFile(_sdf);

  // Open the log file for writing.
  this->logFileStream.open(this->logFilePath.string().c_str(),
      std::fstream::out);
  if (!this->logFileStream.is_open())
  {
    gzerr << "Failed to open log file :" << this->logFilePath << std::endl;
    return;
  }
  std::cout << "[SRCLog plugin] Writing log data to " << this->logFilePath
            << std::endl;

  // Write the log file header.
  this->logFileStream << "# Format: " << std::endl;
  this->logFileStream << "# wallTime(sec) simTime(sec) "
    "model_0_name model_0_pose ... model_N-1_name model_N-1_pose \"custom_log\""
    << std::endl;

  // Listen to the update event. This event is broadcast every sim iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LogPlugin::OnUpdate, this, std::placeholders::_1));
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

    std::string logTimeDir = gazebo::common::Time::GetWallTimeAsISOString();

    this->logFilePath /= ".gazebo";
    this->logFilePath /= "log";
    this->logFilePath /= logTimeDir;
    this->logFilePath /= this->world->GetName() + ".src.log";
  }

  // Create the log directory if needed.
  if (!boost::filesystem::exists(this->logFilePath.parent_path()))
    boost::filesystem::create_directories(this->logFilePath.parent_path());
}

/////////////////////////////////////////////////
void LogPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Collect timing information.
  common::Time wallTime = common::Time::GetWallTime();
  common::Time simTime = _info.simTime;

  // Collect model information.
  std::ostringstream modelInfo;
  for (const auto model : this->world->GetModels())
    modelInfo << " " << model->GetName() << " " << model->GetWorldPose();

  // Collect custom information depending on the world (e.g.: SRC_qual1).
  std::string customLog;
  switch (this->worldType)
  {
    case QUAL_1:
      customLog = this->LogQual1();
      break;
    case QUAL_2:
      customLog = this->LogQual2();
      break;
    case FINAL_1:
      customLog = this->LogFinal1();
      break;
    case FINAL_2:
      customLog = this->LogFinal2();
      break;
    case FINAL_3:
      customLog = this->LogFinal3();
      break;
    default:
      GZ_ASSERT(false, "Unknown worldType");
  }

  std::string logEntry = modelInfo.str() + "\"" + customLog + "\"";

  this->WriteLog(wallTime, simTime, logEntry);
}

/////////////////////////////////////////////////
void LogPlugin::WriteLog(const common::Time &_wallTime,
  const common::Time &_simTime, const std::string &_msg)
{
  if (!this->logFileStream.is_open())
  {
    gzerr << "Log file stream is no longer open:" << this->logFilePath <<
        std::endl;
    return;
  }

  this->logFileStream << std::fixed << std::setprecision(3)
    << _wallTime.Double() << " "
    << _simTime.Double()
    << _msg << std::endl;
}

/////////////////////////////////////////////////
std::string LogPlugin::LogQual1() const
{
  // Save the RGB value and position of LEDs in Valkerie's head.
  std::string res;
  return res;
}

/////////////////////////////////////////////////
std::string LogPlugin::LogQual2() const
{
  // Save the position of the button.

  // Save whether the door was opened.

  std::string res;
  return res;
}

/////////////////////////////////////////////////
std::string LogPlugin::LogFinal1() const
{
  std::string res;
  return res;
}

/////////////////////////////////////////////////
std::string LogPlugin::LogFinal2() const
{
  std::string res;
  return res;
}

/////////////////////////////////////////////////
std::string LogPlugin::LogFinal3() const
{
  std::string res;
  return res;
}
