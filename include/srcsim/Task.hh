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

#ifndef SRC_TASK_HH_
#define SRC_TASK_HH_

#include <memory>
#include <vector>

#include <ros/ros.h>

#include <sdf/sdf.hh>

#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>

#include <gazebo/transport/transport.hh>

#include <std_msgs/Empty.h>

#include "Checkpoint.hh"

namespace gazebo
{
  /// \brief Base class for all 3 tasks.
  class Task
  {
    /// \brief Constructor
    /// \param[in] _sdf SDF element for this task.
    public: Task(const sdf::ElementPtr &_sdf);

    /// \brief Start this task at a specific checkpoint
    /// \param[in] _time Start time
    /// \param[in] _checkpoint Checkpoint Id
    public: void Start(const common::Time &_time, const size_t _checkpoint);

    /// \brief Update this task
    /// \param[in] _time Current time
    public: void Update(const common::Time &_time);

    /// \brief Skip this task
    /// \param[in] _penalty True if skip penalty should be applied.
    public: void Skip(const bool _penalty);

    /// \brief Return the number of checkpoints in this task.
    /// \return Number of checkpoints.
    public: size_t CheckpointCount() const;

    /// \brief Return the index of the current checkpoint.
    /// \return Index of current checkpoint.
    public: size_t CurrentCheckpointId() const;

    /// \brief Return the total time taken to complete a checkpoint.
    /// \param[in] _index Index of the checkpoint in the array (0~n-1)
    /// \return The competion time, zero if it wasn't complete.
    public: common::Time GetCheckpointCompletion(const size_t index) const;

    /// \brief Return the total time penalty taken by a checkpoint.
    /// \param[in] _index Index of the checkpoint in the array (0~n-1)
    /// \return The sum of all penalties for a checkpoint.
    public: common::Time GetCheckpointPenalty(const size_t index) const;

    /// \brief Return this task's number.
    /// \return Task number.
    public: virtual size_t Number() const = 0;

    /// \brief Skip all remaining checkpoints up to the given one, inclusive.
    /// \param[in] _lastSkipped Id of last checkpoint to be skipped (1..n).
    /// \param[in] _penalty True if skip penalty should be applied.
    public: void SkipUpTo(const size_t _lastSkipped, const bool _penalty);

    /// \brief Callback when messages are received from the BoxContainsPlugin.
    /// \param[in] _msg 1 if robot is inside box, 0 otherwise.
    private: void OnStartBox(ConstIntPtr &_msg);

    /// \brief Increment penalty time for current checkpoint, based on current
    /// `previousPenalty`.
    private: void ApplyPenaltyTime();

    /// \brief Vector of checkpoints for this task.
    /// checkpoints[0]: Checkpoint 1
    /// checkpoints[1]: Checkpoint 2
    /// checkpoints[2]: Checkpoint 3
    /// ...
    protected: std::vector<std::unique_ptr<Checkpoint> > checkpoints;

    /// \brief Current checkpoint number, starting from 1. Zero means the task
    /// hasn't started, Count+1 means that the task has finished.
    private: size_t current = 0;

    /// \brief Vector of total times each checkpoint took to complete. That
    /// doesn't include time penalties.
    /// Time is zero for skipped checkpoints.
    private: std::vector<common::Time> cpDuration;

    /// \brief Time when the task started
    private: common::Time startTime;

    /// \brief Total time allowed for the task.
    private: common::Time timeout = 300;

    /// \brief True if task has been completed before timeout.
    private: bool finished = false;

    /// \brief True if task has timed out before completion.
    private: bool timedOut = false;

    /// \brief Total time penalty due to resets and skips
    private: common::Time totalPenalty;

    /// \brief Previous time penalty due to resets and skips. This carries
    /// over from one task to the next.
    private: static common::Time previousPenalty;

    /// \brief Ros node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief Ros publisher which publishes the task's status.
    private: ros::Publisher taskRosPub;

    /// \brief Gazebo transport node for communication.
    protected: transport::NodePtr gzNode;

    /// \brief Gazebo subscriber of box contains messages.
    private: transport::SubscriberPtr boxSub;

    /// \brief Gazebo publisher of toggle messages.
    private: transport::PublisherPtr togglePub;

    /// \brief Flag to indicate whether the robot has reached the start box.
    private: bool startBox = false;

    /// \brief Mutex used to protect the update loop
    private: std::mutex updateMutex;

    /// \brief Ros force checkpoint completion subscriber
    private: ros::Subscriber forceCpCompletionRosSub;

    /// \brief Callback when a force checkpoint completion message is received.
    /// \param[in] _msg Force checkpoint completion message
    private: void OnForceCpCompletionRosMsg(
        const std_msgs::Empty::ConstPtr &_msg);

    /// \brief Flag to indicate forcing checkpoint completion.
    private: bool forceCpCompletion = false;
  };
}
#endif
