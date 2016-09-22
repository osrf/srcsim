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
#ifndef GAZEBO_PLUGINS_QUAL2PLUGIN_HH_
#define GAZEBO_PLUGINS_QUAL2PLUGIN_HH_

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE Qual2Plugin : public ModelPlugin
  {
    /// \brief Constructor
    public: Qual2Plugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Whether the button is pressed.
    protected: bool Pressed() const;

    /// \brief Open the door.
    protected: void OpenTheDoor();

    /// \brief Update plugin's function.
    private: void OnUpdate();

    /// \brief Pointer to the joint that actuates the button.
    protected: physics::JointPtr buttonJoint;

    /// \brief Pointer to the joint that opens/closes the door (the hinge).
    protected: physics::JointPtr hingeJoint;

    /// \brief Whether the button is pressed.
    protected: bool pressed = false;

    /// \brief Lower limit of the button's joint.
    protected: double lowerLimit;

    /// \brief Upper limit of the button's joint.
    protected: double upperLimit;

    /// \brief The range of motion of the button's joint.
    protected: double range;

    /// \brief The door is opening.
    protected: bool opening = false;

    /// \brief Between 0% of the range and this value the button is considered
    /// not pressed. From this value to 100% the button is considered pressed.
    protected: static const int kPercentageButtonPressed = 75;

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;
  };
}

#endif
