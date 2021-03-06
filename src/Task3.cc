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

#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <srcsim/Leak.h>

#include "geometry_msgs/Point.h"
#include "srcsim/HarnessManager.hh"
#include "srcsim/Task3.hh"

using namespace gazebo;

// Pose of the detector on the table
ignition::math::Pose3d detectorOnTable;

// Pose of the patch tool on the table
ignition::math::Pose3d patchToolOnTable;

// Leak model name
std::string leakModelName;

/////////////////////////////////////////////////
// Teleport detector to table
void toolsToTable()
{
  auto world = physics::get_world();
  if (!world)
  {
    gzerr << "Failed to get world" << std::endl;
    return;
  }

  auto detector = world->GetModel("air_leak_detector");
  if (!detector)
  {
    gzerr << "Failed to get [air_leak_detector] model" << std::endl;
    return;
  }

  auto tool = world->GetModel("leak_patch_tool");
  if (!tool)
  {
    gzerr << "Failed to get [leak_patch_tool] model" << std::endl;
    return;
  }

  detector->SetWorldPose(detectorOnTable);
  tool->SetWorldPose(patchToolOnTable);
}

/////////////////////////////////////////////////
// Unlock and push door
void openDoor()
{
  // Remove lock
  auto world = physics::get_world();

  if (!world)
  {
    gzerr << "Failed to get world" << std::endl;
    return;
  }

  auto door = world->GetModel("habitat_door");
  if (!door)
  {
    gzerr << "Failed to get model [habitat_door]" << std::endl;
    return;
  }

  door->RemoveJoint("door_lock");

  // Push door
  auto hingeJoint = door->GetJoint("door_hinge");
  if (!hingeJoint)
  {
    gzerr << "Failed to get joint [hinge_joint]" << std::endl;
    return;
  }

  auto angle = hingeJoint->GetAngle(0).Radian();
  hingeJoint->SetForce(0, 100000 * (1.0 - angle / (M_PI * 0.5)));
}

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

    if (_sdf->HasElement("detector_on_table"))
      detectorOnTable = _sdf->Get<ignition::math::Pose3d>("detector_on_table");

    if (_sdf->HasElement("patch_tool_on_table"))
    {
      patchToolOnTable =
          _sdf->Get<ignition::math::Pose3d>("patch_tool_on_table");
    }

    if (_sdf->HasElement("leak_model_name"))
    {
      leakModelName = _sdf->Get<std::string>("leak_model_name");
    }
    else
    {
      gzerr << "Missing <leak_model_name>, task won't be initialized."
            << std::endl;
      return;
    }
  }

  // Checkpoint 1: Climb the stairs
  std::unique_ptr<Task3CP1> cp1(new Task3CP1(cp1Elem));
  this->checkpoints.push_back(std::move(cp1));

  // Checkpoint 2: Open the door
  std::unique_ptr<Task3CP2> cp2(new Task3CP2(cp2Elem));
  this->checkpoints.push_back(std::move(cp2));

  // Checkpoint 3: Pass through the door
  std::unique_ptr<Task3CP3> cp3(new Task3CP3(cp3Elem));
  this->checkpoints.push_back(std::move(cp3));

  // Checkpoint 4: Lift detector
  std::unique_ptr<Task3CP4> cp4(new Task3CP4(cp4Elem));
  this->checkpoints.push_back(std::move(cp4));

  // Checkpoint 5: Detect leak
  std::unique_ptr<Task3CP5> cp5(new Task3CP5(cp5Elem));
  this->checkpoints.push_back(std::move(cp5));

  // Checkpoint 6: Lift patch tool
  std::unique_ptr<Task3CP6> cp6(new Task3CP6(cp6Elem));
  this->checkpoints.push_back(std::move(cp6));

  // Checkpoint 7: Patch leak
  std::unique_ptr<Task3CP7> cp7(new Task3CP7(cp7Elem));
  this->checkpoints.push_back(std::move(cp7));

  // Checkpoint 8: Walk to final box
  std::unique_ptr<Task3CP8> cp8(new Task3CP8(cp8Elem));
  this->checkpoints.push_back(std::move(cp8));

  this->logFilter =
      "air_leak_detector*|habitat_door*|leak_patch_tool*|valkyrie*";
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
void Task3CP1::Restart(const common::Time &_penalty)
{
  // This is the 1st CP of the task: reharness back at start box
  HarnessManager::Instance()->NewGoal(this->robotStartPose);

  Checkpoint::Restart(_penalty);
}

/////////////////////////////////////////////////
bool Task3CP2::Check()
{
  // First time
  if (!this->valveJoint || !this->hingeJoint)
  {
    this->Start();

    auto world = physics::get_world();

    if (!world)
    {
      gzerr << "Failed to get world" << std::endl;
      return false;
    }

    this->model = world->GetModel("habitat_door");
    if (!this->model)
    {
      gzerr << "Failed to get model [habitat_door]" << std::endl;
      return false;
    }

    this->valveJoint = model->GetJoint("valve_hinge");
    this->hingeJoint = model->GetJoint("door_hinge");
    if (!this->valveJoint || !this->hingeJoint)
    {
      gzerr << "Failed to get some joint" << std::endl;
      return false;
    }
  }

  // If valve is turned, remove lock
  if (!this->unlocked && this->valveJoint->GetAngle(0) > this->valveTarget)
  {
    this->model->RemoveJoint("door_lock");
    this->unlocked = true;
    gzmsg << "Task [3] - Checkpoint [2] - Door unlocked" << std::endl;
  }

  if (!this->unlocked)
    return false;

  // Check if door is open
  return this->hingeJoint->GetAngle(0) > this->hingeTarget;
}

/////////////////////////////////////////////////
void Task3CP2::Skip()
{
  openDoor();

  Checkpoint::Skip();
}

/////////////////////////////////////////////////
bool Task3CP3::Check()
{
  return this->CheckBox("/task3/checkpoint3");
}

/////////////////////////////////////////////////
void Task3CP3::Restart(const common::Time &_penalty)
{
  openDoor();

  Checkpoint::Restart(_penalty);
}

/////////////////////////////////////////////////
bool Task3CP4::Check()
{
  return this->CheckTouch("/task3/checkpoint4");
}

/////////////////////////////////////////////////
void Task3CP4::Restart(const common::Time &_penalty)
{
  toolsToTable();

  Checkpoint::Restart(_penalty);
}

/////////////////////////////////////////////////
Task3CP5::~Task3CP5()
{
  if (!this->stopLeakPosePub && this->leakPoseThread)
  {
    this->stopLeakPosePub = true;
    this->leakPoseThread->join();
  }
}

/////////////////////////////////////////////////
bool Task3CP5::Check()
{
  // First time
  if (!this->cameraGzSub)
  {
    this->Start();

    // Gazebo node
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Subscribe to logical camera messages
    this->cameraGzSub = this->gzNode->Subscribe(this->cameraGzTopic,
        &Task3CP5::OnCameraGzMsg, this);

    // ROS node
    if (!this->rosNode)
    {
      this->rosNode.reset(new ros::NodeHandle());
    }

    // Publish ROS leak messages
    this->leakRosPub = this->rosNode->advertise<srcsim::Leak>(
        "/task3/checkpoint5/leak", 1000);

    // Calculate factor

    // Furthest detectable point (frustum corner)
    double c = this->camFar * tan(this->camFov / 2.0);
    ignition::math::Vector3d corner(camFar, c, c);

    // Maximum detectable distance
    ignition::math::Vector3d antenaPos(this->camNear, 0, 0);
    auto maxDist = corner.Distance(antenaPos);

    // output = factor ^ distance
    // We chose the factor so that the maximum distance results in the
    // minimum value.
    this->factor = pow(this->minValue, 1 / maxDist);
  }

  return this->detected;
}

/////////////////////////////////////////////////
void Task3CP5::Restart(const common::Time &_penalty)
{
  toolsToTable();

  Checkpoint::Restart(_penalty);
}

/////////////////////////////////////////////////
void Task3CP5::PublishLeakPose()
{
  auto world = physics::get_world();

  if (!world)
  {
    gzerr << "Failed to get world" << std::endl;
    return;
  }

  if (!world->GetModel("valkyrie"))
  {
    gzerr << "Missing valkyrie model\n";
    return;
  }

  if (!world->GetModel("valkyrie")->GetLink("pelvis"))
  {
    gzerr << "Valkyrie is missing the pelvis link\n";
    return;
  }

  // Wait for the leak model to appear
  for (int i = 0; i < 50 && !world->GetModel(leakModelName); ++i)
  {
    gazebo::common::Time::MSleep(500);
  }

  // Make sure it exists.
  if (!world->GetModel(leakModelName))
  {
    gzerr << "Missing leak model\n";
    return;
  }

  if (!this->rosNode)
  {
    this->rosNode.reset(new ros::NodeHandle());
  }

  // Publisher for ROS leak pose messages
  this->leakPoseRosPub = this->rosNode->advertise<geometry_msgs::Point>(
      "/task3/checkpoint5/leak_pose", 1000);

  auto pelvis = world->GetModel("valkyrie")->GetLink("pelvis");
  auto leak = world->GetModel(leakModelName);

  // Keep publising the pose of the leak model in the pelvis frame
  while (!this->stopLeakPosePub)
  {
    auto pelvisWorldPose = pelvis->GetWorldPose();
    auto leakWorldPose = leak->GetWorldPose();
    auto leakPoseInPelvisFrame = leakWorldPose - pelvisWorldPose;

    geometry_msgs::Point msg;
    msg.x = leakPoseInPelvisFrame.pos.x;
    msg.y = leakPoseInPelvisFrame.pos.y;
    msg.z = leakPoseInPelvisFrame.pos.z;

    this->leakPoseRosPub.publish(msg);
    gazebo::common::Time::MSleep(1000);
  }
}

/////////////////////////////////////////////////
void Task3CP5::Skip()
{
  // Create a thread that will publish leak pose information until the end
  // of the round.
  this->stopLeakPosePub = false;
  this->leakPoseThread = new std::thread(&Task3CP5::PublishLeakPose, this);
  Checkpoint::Skip();
}

//////////////////////////////////////////////////
void Task3CP5::OnCameraGzMsg(ConstLogicalCameraImagePtr &_msg)
{
  auto leakPos = ignition::math::Vector3d::Zero;

  for (const auto &model : _msg->model())
  {
    if (model.name() != leakModelName)
      continue;

    leakPos = msgs::ConvertIgn(model.pose().position());
    break;
  }

  double value = this->minValue;

  // If leak within frustum
  if (leakPos != ignition::math::Vector3d::Zero)
  {
    // Leak found, checkpoint complete!
    this->detected = true;

    // Get distance from leak to antena
    ignition::math::Vector3d antenaPos(this->camNear, 0, 0);
    auto distance = leakPos.Distance(antenaPos);

    // Exclude some partial inclusion
    if (leakPos.X() >= this->camNear)
      value = std::max(pow(this->factor, distance), this->minValue);
  }

  // We keep publishing messages even after complete in case they want to get a
  // better position
  srcsim::Leak msg;
  msg.value = value;

  this->leakRosPub.publish(msg);
}

/////////////////////////////////////////////////
bool Task3CP6::Check()
{
  return this->CheckTouch("/task3/checkpoint6");
}

/////////////////////////////////////////////////
void Task3CP6::Restart(const common::Time &_penalty)
{
  toolsToTable();

  Checkpoint::Restart(_penalty);
}

/////////////////////////////////////////////////
bool Task3CP7::Check()
{
  // First time
  if (!this->sensor || !this->buttonJoint)
  {
    this->Start();

    this->world = physics::get_world();

    if (!this->world)
    {
      gzerr << "Failed to get world" << std::endl;
      return false;
    }
    // Get contact sensor
    auto s = sensors::SensorManager::Instance()->GetSensor(this->sensorName);
    if (!s)
    {
      gzerr << "Failed to get sensor [" << this->sensorName << "]" << std::endl;
      return false;
    }

    this->sensor = std::dynamic_pointer_cast<sensors::ContactSensor>(s);
    if (!this->sensor)
    {
      gzerr << "Failed to cast sensor [" << this->sensorName << "]" << std::endl;
      return false;
    }

    this->sensor->SetActive(true);

    // Get button joint
    auto toolModel = world->GetModel(this->tool);
    if (!toolModel)
    {
      gzerr << "Failed to get model [" << this->tool << "]" << std::endl;
      return false;
    }

    this->buttonJoint = toolModel->GetJoint(this->buttonName);
    if (!this->buttonJoint)
    {
      gzerr << "Failed to get joint [" << this->buttonJoint << "]"
            << std::endl;
      return false;
    }

    if (!leakModelName.empty())
    {
      this->leak = leakModelName + "::base::collision";
    }
    else
    {
      gzerr << "Missing leakModelName, can't patch leak!" << std::endl;
      return false;
    }
  }

  auto simTime = this->world->GetSimTime();

  // Check if button is pressed
  if (this->buttonJoint->GetAngle(0) > this->buttonTarget)
  {
    // Stopped pressing
    if (this->fixStart != common::Time::Zero)
    {
      gzmsg << "Released button at " << simTime << " seconds" << std::endl;
    }

    this->fixStart = common::Time::Zero;
    return false;
  }

  // Check if tool is touching leak
  bool touching = false;

  // Go through all contacts
  auto contacts = this->sensor->Contacts();

  for (auto const &contact : contacts.contact())
  {
    // Check for the tool
    bool col1Tool = contact.collision1().find(this->toolTip) !=
        std::string::npos;
    bool col2Tool = contact.collision2().find(this->toolTip) !=
        std::string::npos;

    // Check for the leak
    bool col1Leak = contact.collision1().find(this->leak) !=
        std::string::npos;
    bool col2Leak = contact.collision2().find(this->leak) !=
        std::string::npos;

    // We need at least one tool-leak or leak-tool contact
    if ((col1Tool && col2Leak) || (col1Leak && col2Tool))
    {
      touching = true;
      break;
    }
  }

  // Not touching
  if (!touching)
  {
    // Stopped touching
    if (this->fixStart != common::Time::Zero)
    {
      gzmsg << "Tool stopped touching leak at " << simTime
            << " seconds" << std::endl;
    }
    this->fixStart = common::Time::Zero;
    return false;
  }

  // Just started touching
  if (this->fixStart == common::Time::Zero)
  {
    this->fixStart = simTime;
    // This is only printed if button is also pressed.
    gzmsg << "Tool started touching leak at " << this->fixStart
          << " seconds" << std::endl;
  }

  // Check if it has been pressing for long enough
  return simTime - this->fixStart > this->targetTime;
}

/////////////////////////////////////////////////
void Task3CP7::Restart(const common::Time &_penalty)
{
  toolsToTable();

  Checkpoint::Restart(_penalty);
}

/////////////////////////////////////////////////
bool Task3CP8::Check()
{
  return this->CheckBox("/task3/checkpoint8");
}

