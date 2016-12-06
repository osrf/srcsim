/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>

namespace gazebo
{
  class SatelliteDishPlugin : public ModelPlugin
  {
    public: SatelliteDishPlugin() : ModelPlugin()
      {

      }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {

        // Read in the ratios from SDF
        if (_sdf->HasElement("yaw_ratio"))
        {
          this->yawRatio = _sdf->Get<double>("yaw_ratio");
        }
        else
        {
          gzerr << "Missing [yaw_ratio] parameter, plugin will not load."
                << std::endl;
          return;
        }

        if (_sdf->HasElement("pitch_ratio"))
        {
          this->pitchRatio = _sdf->Get<double>("pitch_ratio");
        }
        else
        {
          gzerr << "Missing [pitch_ratio] parameter, plugin will not load."
                << std::endl;
          return;
        }

        if (_sdf->HasElement("frequency"))
        {
          this->frequency = _sdf->Get<double>("frequency");
        }
        else
        {
          this->frequency = 100;
        }

        // Get joints
        this->yawJointName = _model->GetJoint("dish_yaw")->GetScopedName();
        this->pitchJointName = _model->GetJoint("dish_pitch")->GetScopedName();
        this->yawWheel = _model->GetJoint("wheel_yaw");
        this->pitchWheel = _model->GetJoint("wheel_pitch");

        if (!this->yawWheel || !this->pitchWheel)
        {
          gzerr << "Missing joint in model, plugin will not load."
                << std::endl;
          return;
        }

        // Configure controller
        this->controller = _model->GetJointController();

        if (_sdf->HasElement("yaw_pid"))
        {
          auto vec = _sdf->Get<ignition::math::Vector3d>("yaw_pid");

          common::PID pid(vec.X(), vec.Y(), vec.Z());
          this->controller->SetPositionPID(this->yawJointName, pid);
        }
        else
        {
          gzerr << "Missing [yaw_pid] parameter, plugin will not load."
                << std::endl;
          return;
        }

        if (_sdf->HasElement("pitch_pid"))
        {
          auto vec = _sdf->Get<ignition::math::Vector3d>("pitch_pid");

          common::PID pid(vec.X(), vec.Y(), vec.Z());
          this->controller->SetPositionPID(this->pitchJointName, pid);
        }
        else
        {
          gzerr << "Missing [pitch_pid] parameter, plugin will not load."
                << std::endl;
          return;
        }

        // Create joint to the world
        physics::JointPtr joint;
        joint = _model->GetWorld()->GetPhysicsEngine()->CreateJoint("fixed", _model);
        joint->Load(nullptr, _model->GetLink("base"), ignition::math::Pose3d());
        joint->Attach(nullptr, _model->GetLink("base"));

        this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
            std::bind(&SatelliteDishPlugin::OnUpdate, this,
            std::placeholders::_1)));

      }

    public: void OnUpdate(const common::UpdateInfo &_info)
      {
        static auto lastUpdate = common::Time::Zero;

        // Limit update rate for performance
        if (_info.simTime - lastUpdate < 1/this->frequency)
          return;

        lastUpdate = _info.simTime;

        this->controller->SetPositionTarget(this->yawJointName,
            this->yawWheel->GetAngle(0).Radian() * this->yawRatio);
        this->controller->SetPositionTarget(this->pitchJointName,
            this->pitchWheel->GetAngle(0).Radian() * this->pitchRatio);
      }

    private: double yawRatio;
    private: double pitchRatio;
    private: double frequency;
    private: std::string yawJointName;
    private: std::string pitchJointName;
    private: physics::JointPtr yawWheel;
    private: physics::JointPtr pitchWheel;
    private: physics::JointControllerPtr controller;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;
  };
  GZ_REGISTER_MODEL_PLUGIN(SatelliteDishPlugin)
}
