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

#ifndef SRC_HARNESSMANAGER_HH_
#define SRC_HARNESSMANAGER_HH_

#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include <gazebo/common/SingletonT.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
  /// \brief Manages topics and their subscriptions
  class HarnessManager : public SingletonT<HarnessManager>
  {
    /// \brief Constructor
    private: HarnessManager();

    /// \brief Destructor
    private: virtual ~HarnessManager();

    /// \brief Add a new goal, overriding previous one
    /// \brief _pose Goal pose to be attached
    public: void NewGoal(const ignition::math::Pose3d &_pose);

    /// \brief Update harness state
    /// \brief _time Sim time
    public: void Update(const common::Time &_time);

    public: void TriggerDetach();
    public: void TriggerAttach();
    public: void TriggerLower();
    public: void TriggerStand();
    public: bool IsDetached();
    public: bool IsAttached();
    public: bool IsLowered();
    public: bool IsStanding();
    private: void OnSensorMsg(ConstWrenchStampedPtr &_msg);

    private: enum Transition
    {
      NONE,
      DETACH_TO_ATTACH,
      ATTACH_TO_LOWER,
      LOWER_TO_STAND,
      STAND_TO_DETACH,
      DETACH_TO_NONE,
    };

    private: Transition transition = NONE;
    private: ignition::math::Pose3d goal;
    private: bool goalChanged = false;
    private: gazebo::physics::ModelPtr model;
    private: double lastSensor = 0.0;
    private: const unsigned int itThreshold = 500;
    private: unsigned int itLowering = 0;
    private: unsigned int itStanding = 0;

    private: gazebo::transport::NodePtr gzNode;
    private: gazebo::transport::PublisherPtr attachGzPub;
    private: gazebo::transport::PublisherPtr detachGzPub;
    private: gazebo::transport::PublisherPtr lowerGzPub;
    private: gazebo::transport::SubscriberPtr sensorGzSub;

    /// \brief Ros node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief Ros publisher for control messages.
    private: ros::Publisher controlRosPub;

    // Singleton implementation
    private: friend class SingletonT<HarnessManager>;
  };
  /// \}
}
#endif
