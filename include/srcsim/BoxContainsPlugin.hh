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

#include <ignition/math/OrientedBox.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  /// \brief Plugin which emits ignition transport messages according to whether
  /// an entity is inside or outside an oriented box.
  ///
  /// Example usage:
  ///
  ///  <plugin name="boxContainsRobot" filename="libBoxContainsPlugin.so">
  ///
  ///    <!-- True to start checking automatically, false so it only starts
  ///         when toggled -->
  ///    <start>true</start>
  ///
  ///    <!-- Frequency in Hertz on which to send updates -->
  ///    <frequency>100</frequency>
  ///
  ///    <!-- Scoped name of entity to check -->
  ///    <entity>robot</entity>
  ///
  ///    <!-- Namespace for ignition transport topic and service:
  ///           /<namespace>/box/contains : topic where true / false messages
  ///                                       are published.
  ///           /<namespace>/box/toggle : service to toggle plugin on and off.
  ///    -->
  ///    <namespace>gazebo/robot</namespace>
  ///
  ///    <!-- Box size in meters -->
  ///    <size>1 1 4</size>
  ///
  ///    <!-- Pose of the box center point in world coordinates -->
  ///    <pose>10 10 2 0 0 1.57</pose>
  ///
  ///  </plugin>
  ///
  class BoxContainsPlugin : public WorldPlugin
  {
    // Documentation inherited
    public: BoxContainsPlugin();

    // Documentation inherited
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Called every world iteration on world update begin.
    /// \param[in] _info Update info.
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Callback for toggle service.
    /// \param[in] _rep Response, either "Started" or "Stoped".
    /// \param[in] _result Whether the service succeeded.
    public: void Toggle(ignition::msgs::StringMsg &_rep, bool &_result);

    /// \brief Connection to world update.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the world.
    private: physics::WorldPtr world;

    /// \brief Scoped name of the entity we're checking.
    private: std::string entityName;

    /// \brief Pointer to the entity we're checking.
    private: physics::EntityPtr entity;

    /// \brief Box representing the volume to check.
    private: ignition::math::OrientedBoxd box;

    /// \brief Ignition transport node for communication.
    private: ignition::transport::Node ignNode;

    /// \brief Publisher which publishes contain / doesn't contain messages.
    private: ignition::transport::Node::Publisher containsPub;

    /// \brief Namespace for the topics:
    /// /<ns>/box/contains
    /// /<ns>/box/toggle
    private: std::string ns;

    /// \brief Update period in seconds.
    public: int period;

    /// \brief Previous update time.
    public: common::Time prevTime;
  };
}

