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

#ifndef SRC_TASK3_HH_
#define SRC_TASK3_HH_

#include <vector>

#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>

#include <ros/ros.h>

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/transport.hh>

#include "Checkpoint.hh"
#include "Task.hh"

namespace gazebo
{
  class Task3 : public Task
  {
    /// \brief Constructor
    /// \param[in] _sdf Pointer to SDF element for this task.
    public: Task3(const sdf::ElementPtr &_sdf);

    // Documentation inherited
    public: size_t Number() const;
  };

  /// \brief Task 3, Checkpoint 1: Climb the stairs
  class Task3CP1 : public BoxCheckpoint
  {
    using BoxCheckpoint::BoxCheckpoint;

    /// \brief Check whether the robot is on the top of the stairs.
    public: bool Check();
  };

  /// \brief Task 3, Checkpoint 2: Open the door
  class Task3CP2 : public Checkpoint
  {
    using Checkpoint::Checkpoint;

    /// \brief Check whether the door is open
    public: bool Check();

    /// \brief Pointer to the door model
    private: physics::ModelPtr model;

    /// \brief Pointer to the valve joint
    private: physics::JointPtr valveJoint;

    /// \brief Pointer to the door hinge joint
    private: physics::JointPtr hingeJoint;

    /// \brief Target angle for the valve to be open
    private: ignition::math::Angle valveTarget = IGN_PI * 2.0;

    /// \brief Target angle for the door to be open
    private: ignition::math::Angle hingeTarget = IGN_PI * 0.4;

    /// \brief True if the door has been unlocked
    private: bool unlocked = false;
  };

  /// \brief Task 3, Checkpoint 3: Pass through door
  class Task3CP3 : public BoxCheckpoint
  {
    using BoxCheckpoint::BoxCheckpoint;

    /// \brief Check whether the robot is on the other side of the door
    public: bool Check();
  };

  /// \brief Task 3, Checkpoint 4: Lift detector
  class Task3CP4 : public TouchCheckpoint
  {
    using TouchCheckpoint::TouchCheckpoint;

    /// \brief Check whether the detector has been touching the robot and
    /// nothing else for long enough.
    /// \return True if the checkpoint is complete.
    public: bool Check();
  };

  /// \brief Task 3, Checkpoint 5: Detect leak
  class Task3CP5 : public Checkpoint
  {
    using Checkpoint::Checkpoint;

    /// \brief Check whether the detector has detected the leak.
    /// \return True if the checkpoint is complete.
    public: bool Check();

    /// \brief Callback when a logical camera message is received,
    /// \param[in] _msg Logical camera message.
    private: void OnCameraGzMsg(ConstLogicalCameraImagePtr &/*_msg*/);

    /// \brief Gazebo transport node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Subscribes to logical camera messages.
    private: transport::SubscriberPtr cameraGzSub;

    /// \brief Ros node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief Ros publisher of leak messages
    private: ros::Publisher leakRosPub;

    /// \brief Topic for camera msgs
    private: std::string cameraGzTopic =
        "/gazebo/SRC_finals/air_leak_detector/base/logical_camera/models";

    /// \brief Whether the leak has been detected
    private: bool detected = false;
  };

  /// \brief Task 3, Checkpoint 6: Lift patch tool
  class Task3CP6 : public TouchCheckpoint
  {
    using TouchCheckpoint::TouchCheckpoint;

    /// \brief Check whether the tool has been touching the robot and
    /// nothing else for long enough.
    /// \return True if the checkpoint is complete.
    public: bool Check();
  };

  /// \brief Task 3, Checkpoint 8: Final box
  class Task3CP8 : public BoxCheckpoint
  {
    using BoxCheckpoint::BoxCheckpoint;

    /// \brief Check whether the robot is in the final box region.
    public: bool Check();
  };
}
#endif
