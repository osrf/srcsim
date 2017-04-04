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

#include "srcsim/Task3.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Task3::Task3(const sdf::ElementPtr &_sdf) : Task(_sdf)
{
  gzmsg << "Creating Task [3] ... ";

  // Get SDF for each checkpoint
  sdf::ElementPtr cp1Elem, cp2Elem, cp3Elem, cp4Elem, cp5Elem, cp6Elem, cp7Elem,
      cp8Elem;
  if (_sdf)
  {
    if (_sdf->HasElement("checkpoint1"))
      cp1Elem = _sdf->GetElement("checkpoint1");

    if (_sdf->HasElement("checkpoint2"))
      cp2Elem = _sdf->GetElement("checkpoint2");

    if (_sdf->HasElement("checkpoint3"))
      cp3Elem = _sdf->GetElement("checkpoint3");

    if (_sdf->HasElement("checkpoint4"))
      cp4Elem = _sdf->GetElement("checkpoint4");

    if (_sdf->HasElement("checkpoint5"))
      cp5Elem = _sdf->GetElement("checkpoint5");

    if (_sdf->HasElement("checkpoint6"))
      cp6Elem = _sdf->GetElement("checkpoint6");

    if (_sdf->HasElement("checkpoint7"))
      cp7Elem = _sdf->GetElement("checkpoint7");

    if (_sdf->HasElement("checkpoint8"))
      cp8Elem = _sdf->GetElement("checkpoint8");
  }

  // Checkpoint 1: Climb the stairs
  std::unique_ptr<Task3CP1> cp1(new Task3CP1(cp1Elem));
  this->checkpoints.push_back(std::move(cp1));

  // Checkpoint 3: Pass through the door
  std::unique_ptr<Task3CP3> cp3(new Task3CP3(cp3Elem));
  this->checkpoints.push_back(std::move(cp3));

  // Checkpoint 4: Lift detector
  std::unique_ptr<Task3CP4> cp4(new Task3CP4(cp4Elem));
  this->checkpoints.push_back(std::move(cp4));

  // Checkpoint 6: Lift patch tool
  std::unique_ptr<Task3CP6> cp6(new Task3CP6(cp6Elem));
  this->checkpoints.push_back(std::move(cp6));

  // Checkpoint 8: Walk to final box
  std::unique_ptr<Task3CP8> cp8(new Task3CP8(cp8Elem));
  this->checkpoints.push_back(std::move(cp8));

  gzmsg << "Task [3] created" << std::endl;
}

/////////////////////////////////////////////////
size_t Task3::Number() const
{
  return 3u;
}

/////////////////////////////////////////////////
bool Task3CP1::Check()
{
  return this->CheckBox("/task3/checkpoint1");
}

/////////////////////////////////////////////////
bool Task3CP3::Check()
{
  return this->CheckBox("/task3/checkpoint3");
}

/////////////////////////////////////////////////
bool Task3CP4::Check()
{
  return this->CheckTouch("/task3/checkpoint4");
}

/////////////////////////////////////////////////
bool Task3CP6::Check()
{
  return this->CheckTouch("/task3/checkpoint6");
}

/////////////////////////////////////////////////
bool Task3CP8::Check()
{
  return this->CheckBox("/task3/checkpoint8");
}

