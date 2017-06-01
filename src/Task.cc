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

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <srcsim/Task.h>
#include <std_msgs/Time.h>

#include "srcsim/Task.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Task::Task(const sdf::ElementPtr &_sdf)
{
  // Get timeout
  if (_sdf && _sdf->HasElement("timeout"))
  {
    this->timeout = _sdf->Get<double>("timeout");
  }
  else
  {
    gzwarn << "Timeout not specified, using default value ["
           << this->timeout << "]" << std::endl;
  }

  // ROS transport
  this->rosNode.reset(new ros::NodeHandle());

  this->taskRosPub = this->rosNode->advertise<srcsim::Task>(
      "/srcsim/finals/task", 1000);
}

/////////////////////////////////////////////////
void Task::Start(const common::Time &_time, const size_t _checkpoint)
{
  std::lock_guard<std::mutex> lock(this->updateMutex);
  // Double-check that we're not going back to a previous checkpoint
  if (_checkpoint < this->current)
  {
    gzerr << "Trying to start task [" << unsigned(this->Number()) <<
        "] checkpoint [" << unsigned(_checkpoint) <<
        "], and current checkpoint is [" << unsigned(this->current) << "]. " <<
        "It's not possible to go back to a previous checkpoint." << std::endl;
    return;
  }

  // Restarting current checkpoint
  if (_checkpoint == this->current)
  {
    // Trigger skip on previous cp to rearrange world
    if (this->current > 1)
      this->checkpoints[this->current - 2]->Skip();

    // Flag this checkpoint as restarted (for scoring)
    this->checkpoints[this->current - 1]->Restart();

    return;
  }

  // First checkpoint officially starts when entering start box
  if (_checkpoint == 1 && !this->startBox)
  {
    // Already waiting
    if (this->gzNode)
      return;

    // Initialize node
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Setup contains subscriber
    this->boxSub = this->gzNode->Subscribe(
        "/task" + std::to_string(this->Number()) + "/start/box/contains",
        &Task::OnStartBox, this);

    // Setup toggle publisher
    this->togglePub = this->gzNode->Advertise<msgs::Int>(
        "/task" + std::to_string(this->Number()) + "/start/box/toggle");

    // Toggle box plugin on
    msgs::Int msg;
    msg.set_data(1);
    this->togglePub->Publish(msg);

    gzmsg << "Task [" << this->Number()
          << "] - Started: time will start counting as you leave the box."
          << std::endl;

    return;
  }

  // Starting task
  if (this->current == 0)
    this->startTime = _time;

  // Check if there are checkpoints being skipped
  while (this->current < _checkpoint)
  {
    if (this->current > 0)
    {
      this->checkpoints[this->current - 1]->Skip();

      gzmsg << "Task [" << this->Number() << "] - Checkpoint ["
            << this->current << "] - Skipped (" << _time << ")" << std::endl;
      this->checkpointsCompletion.push_back(common::Time::Zero);
    }

    this->current++;
  }

  // Checkpoint
  this->current = _checkpoint;

  gzmsg << "Task [" << this->Number() << "] - Checkpoint [" << this->current
        << "] - Started (" << _time << ")" << std::endl;
}

/////////////////////////////////////////////////
void Task::Update(const common::Time &_time)
{
  std::lock_guard<std::mutex> lock(this->updateMutex);
  // Task has finished
  if (this->current > this->checkpoints.size())
    return;

  // While in start box (before time starts counting)
  if (this->current < 1)
  {
    // Publish ROS task message with zero CP and start time
    srcsim::Task msg;
    msg.task = this->Number();
    msg.current_checkpoint = this->current;
    msg.start_time.fromSec(0);
    msg.elapsed_time.fromSec(0);

    this->taskRosPub.publish(msg);
    return;
  }

  // Terminate start transport
  if (this->gzNode)
  {
    this->boxSub.reset();
    this->togglePub.reset();
    this->gzNode->Fini();
    this->gzNode.reset();
  }

  // Timeout
  auto elapsed = _time - this->startTime;
  this->timedOut = !this->finished && elapsed > this->timeout;

  if (this->timedOut)
  {
    elapsed = this->timeout;
    this->current = this->checkpoints.size() + 1;
    this->Skip();
  }
  else
  {
    // Check if current checkpoint is complete
    if (this->checkpoints[this->current - 1]->Check())
    {
      gzmsg << "Task [" << this->Number() << "] - Checkpoint [" << this->current
            << "] - Completed (" << _time << ")" << std::endl;

      // Sanity check
      if (this->checkpointsCompletion.size() >= this->checkpoints.size())
        gzerr << "Too many checkpoint completions!" << std::endl;

      this->checkpointsCompletion.push_back(_time);

      this->current++;

      // Finish task if this was the last checkpoint
      this->finished = this->current > this->checkpoints.size();

      // Otherwise, start next checkpoint
      if (!this->finished)
      {
        gzmsg << "Task [" << this->Number() << "] - Checkpoint ["
              << this->current << "] - Started (" << _time << ")" << std::endl;
      }
    }
  }

  // Publish ROS task message
  srcsim::Task msg;
  msg.task = this->Number();
  msg.current_checkpoint = this->current;
  msg.finished = this->finished;
  msg.timed_out = this->timedOut;
  msg.start_time.fromSec(this->startTime.Double());
  msg.elapsed_time.fromSec(elapsed.Double());

  for (const auto &time : this->checkpointsCompletion)
  {
    ros::Time t(time.Double());
    msg.checkpoints_completion.push_back(t);
  }

  this->taskRosPub.publish(msg);
}

/////////////////////////////////////////////////
void Task::Skip()
{
  // Trigger skip on last checkpoint
  this->checkpoints.back()->Skip();
}

/////////////////////////////////////////////////
size_t Task::CheckpointCount() const
{
  return this->checkpoints.size();
}

/////////////////////////////////////////////////
size_t Task::CurrentCheckpointId() const
{
  return this->current;
}

/////////////////////////////////////////////////
common::Time Task::GetCheckpointCompletion(const size_t index) const
{
  if (index < this->checkpointsCompletion.size())
  {
    return this->checkpointsCompletion[index];
  }
  return common::Time::Zero;
}

/////////////////////////////////////////////////
bool Task::GetCheckpointRestarted(const size_t index) const
{
  if (index < this->checkpointsCompletion.size())
  {
    return this->checkpoints[index]->Restarted();
  }
  return false;
}

//////////////////////////////////////////////////
void Task::OnStartBox(ConstIntPtr &_msg)
{
  this->startBox = _msg->data() == 0 ? false : true;

  auto world = physics::get_world();
  if (!world)
  {
    gzerr << "Failed to get world pointer, can't start task."
        << std::endl;
    return;
  }

  // Now the checkpoint starts
  this->Start(world->GetSimTime(), 1);
}

