## SRC Sim

### SRC Sim 0.8.0 (2017-06-12)

* Cheat detect
    * [Pull request 103](https://bitbucket.org/osrf/srcsim/pull-requests/103)

* Non-overlapping habitat floor collisions
    * [Pull request 100](https://bitbucket.org/osrf/srcsim/pull-requests/100)

* set solver tolerance to 0.1 for all worlds
    * [Pull request 91](https://bitbucket.org/osrf/srcsim/pull-requests/91)

* Reset joint positions after harnessing (all except the torso joints)
    * [Pull request 99](https://bitbucket.org/osrf/srcsim/pull-requests/99)

* Leak name variable
    * [Pull request 102](https://bitbucket.org/osrf/srcsim/pull-requests/102)

* Update launch wait times
    * [Pull request 101](https://bitbucket.org/osrf/srcsim/pull-requests/101)

* Polyline collision for 45 degrees walkway
    * [Pull request 97](https://bitbucket.org/osrf/srcsim/pull-requests/97)

* add exec_depend on joint_state_publisher for multisense hack
    * [Pull request 96](https://bitbucket.org/osrf/srcsim/pull-requests/96)


### SRC Sim 0.7.0 (2017-06-07)

* Task 3 changes
    * Task 3: Fix leak height
        * [Pull request 88](https://bitbucket.org/osrf/srcsim/pull-requests/88)

    * Task 3: Fix habitat entrance collision
        * [Pull request 89](https://bitbucket.org/osrf/srcsim/pull-requests/89)

    * Task 3: Fix habitat entrance collision
        * [Pull request 89](https://bitbucket.org/osrf/srcsim/pull-requests/89)

    * Task 3: Start detector plugin enabled to avoid race condition
        * [Pull request 90](https://bitbucket.org/osrf/srcsim/pull-requests/90)

    * Skip task 3 checkpoint 5
        * [Pull request 94](https://bitbucket.org/osrf/srcsim/pull-requests/94)

* Others
    * Use log filtering to decrease log size
        * [Pull request 80](https://bitbucket.org/osrf/srcsim/pull-requests/80)

    * Require gazebo 7.8
        * [Pull request 93](https://bitbucket.org/osrf/srcsim/pull-requests/93)

    * Fix skipping before leaving the start box
        * [Pull request 87](https://bitbucket.org/osrf/srcsim/pull-requests/87)

### SRC Sim 0.6.0 (2017-06-01)

* Task 1 changes
    * No changes to tutorial needed
    * Task 1: Enable / disable satellite plugin on demand
        * [Pull request 85](https://bitbucket.org/osrf/srcsim/pull-requests/85)

* Task 2 changes
    * No changes to tutorial needed
    * Task 2: Insert habitat lamp on demand
        * [Pull request 86](https://bitbucket.org/osrf/srcsim/pull-requests/86)

    * Higher array surface, different cable plug
        * [Pull request 77](https://bitbucket.org/osrf/srcsim/pull-requests/77)

* Task 3 changes
    * No changes to tutorial needed
    * Task 3: Simple shape collisions for valve
        * [Pull request 84](https://bitbucket.org/osrf/srcsim/pull-requests/84)

    * Higher table surface
        * [Pull request 77](https://bitbucket.org/osrf/srcsim/pull-requests/77)

* Others
    * [Task 1 tutorial updated](https://bitbucket.org/osrf/srcsim/wiki/finals_task1)
    * [Task 2 tutorial updated](https://bitbucket.org/osrf/srcsim/wiki/finals_task2)
    * [Task 3 tutorial updated](https://bitbucket.org/osrf/srcsim/wiki/finals_task3)
    * Add publishing of Score message
        * [Pull request 69](https://bitbucket.org/osrf/srcsim/pull-requests/69)

    * Time penalties on skip and reset
        * [Pull request 82](https://bitbucket.org/osrf/srcsim/pull-requests/82)

    * Add ROS subscriber to force current checkpoint completion (for debugging)
        * [Pull request 83](https://bitbucket.org/osrf/srcsim/pull-requests/83)

    * Re-harness
        * [Pull request 79](https://bitbucket.org/osrf/srcsim/pull-requests/79)

    * Wait for ihmc controller to be ready before launching grasp controller
        * [Pull request 81](https://bitbucket.org/osrf/srcsim/pull-requests/81)

    * Logging: remove <sdf> from <plugin>
        * [Pull request 78](https://bitbucket.org/osrf/srcsim/pull-requests/78)

    * Publish multisense transforms
        * [Pull request 72](https://bitbucket.org/osrf/srcsim/pull-requests/72)

### SRC Sim 0.5.0 (2017-05-16)

* Task 1 changes
    * [Task 1 tutorial updated](https://bitbucket.org/osrf/srcsim/wiki/finals_task1)
    * Task 1: Split checkpoint 2
        * [Pull request 70](https://bitbucket.org/osrf/srcsim/pull-requests/70)

* Task 2 changes
    * No changes to tutorial needed
    * Task 2: Teleport solar panel within reach
        * [Pull request 63](https://bitbucket.org/osrf/srcsim/pull-requests/63)

    * Task 2: Solar panel from 5 kg to 3 kg
        * [Pull request 74](https://bitbucket.org/osrf/srcsim/pull-requests/74)

    * Improve solar_panel model contact behavior
        * [Pull request 62](https://bitbucket.org/osrf/srcsim/pull-requests/62)

* Adding Task 3
    * [Task 3 tutorial](https://bitbucket.org/osrf/srcsim/wiki/finals_task3)
    * Task 3: Checkpoints 1, 2, 3, 4
        * [Pull request 59](https://bitbucket.org/osrf/srcsim/pull-requests/59)

    * Task 3, Checkpoint 5: Leak detection
        * [Pull request 64](https://bitbucket.org/osrf/srcsim/pull-requests/64)

    * Task 3: Randomize habitat objects
        * [Pull request 60](https://bitbucket.org/osrf/srcsim/pull-requests/60)

    * Task 3: Simple collision shapes for crates
        * [Pull request 66](https://bitbucket.org/osrf/srcsim/pull-requests/66)

    * Task 3: Fix habitat wall collision
        * [Pull request 65](https://bitbucket.org/osrf/srcsim/pull-requests/65)

    * Task 3: Integration
        * [Pull request 57](https://bitbucket.org/osrf/srcsim/pull-requests/57)

    * Performance: Delete / insert models as we need them in view
        * [Pull request 67](https://bitbucket.org/osrf/srcsim/pull-requests/67)

* Others
    * Fix Hokuyo transform publication
        * [Pull request 72](https://bitbucket.org/osrf/srcsim/pull-requests/72)

    * Launch file and 5 example worlds for finals
        * [Pull request 61](https://bitbucket.org/osrf/srcsim/pull-requests/61)

    * Add a CMake parameter for building messages and services only.
        * [Pull request 71](https://bitbucket.org/osrf/srcsim/pull-requests/71)

    * Allow re-harnessing of valkyrie
        * [Pull request 68](https://bitbucket.org/osrf/srcsim/pull-requests/68)

    * Update runtime dependencies list
        * [Pull request 73](https://bitbucket.org/osrf/srcsim/pull-requests/73)

    * Create a helper setup.bash system file
        * [Pull request 40](https://bitbucket.org/osrf/srcsim/pull-requests/40)
        * [Pull request 76](https://bitbucket.org/osrf/srcsim/pull-requests/76)

### SRC Sim 0.4.0 (2017-04-14)

* Grasping controller
    * [Pull request 51](https://bitbucket.org/osrf/srcsim/pull-requests/51)
    * [Pull request 42](https://bitbucket.org/osrf/srcsim/pull-requests/42)

* Task 3 models
    * [Pull request 37](https://bitbucket.org/osrf/srcsim/pull-requests/37)
    * [Pull request 39](https://bitbucket.org/osrf/srcsim/pull-requests/39)
    * [Pull request 50](https://bitbucket.org/osrf/srcsim/pull-requests/50)
    * [Pull request 52](https://bitbucket.org/osrf/srcsim/pull-requests/52)
    * [Pull request 54](https://bitbucket.org/osrf/srcsim/pull-requests/54)

* Task 2 plugins
    * [Pull request 44](https://bitbucket.org/osrf/srcsim/pull-requests/44)
    * [Pull request 45](https://bitbucket.org/osrf/srcsim/pull-requests/45)
    * [Pull request 47](https://bitbucket.org/osrf/srcsim/pull-requests/47)
    * [Pull request 48](https://bitbucket.org/osrf/srcsim/pull-requests/48)
    * [Pull request 49](https://bitbucket.org/osrf/srcsim/pull-requests/49)

* Task 2 models
    * [Pull request 34](https://bitbucket.org/osrf/srcsim/pull-requests/34)
    * [Pull request 41](https://bitbucket.org/osrf/srcsim/pull-requests/41)
    * [Pull request 43](https://bitbucket.org/osrf/srcsim/pull-requests/43)
    * [Pull request 38](https://bitbucket.org/osrf/srcsim/pull-requests/38)
    * [Pull request 55](https://bitbucket.org/osrf/srcsim/pull-requests/55)

* Task 1 updates
    * [Pull request 53](https://bitbucket.org/osrf/srcsim/pull-requests/53)
    * [Pull request 56](https://bitbucket.org/osrf/srcsim/pull-requests/56)

* Skipping updates
    * [Pull request 46](https://bitbucket.org/osrf/srcsim/pull-requests/46)
    * [Pull request 49](https://bitbucket.org/osrf/srcsim/pull-requests/49)

* Start box for each task
    * [Pull request 49](https://bitbucket.org/osrf/srcsim/pull-requests/49)
    * [Task 1 tutorial](https://bitbucket.org/osrf/srcsim/wiki/finals_task1)
    * [Task 2 tutorial](https://bitbucket.org/osrf/srcsim/wiki/finals_task2)

* Match ihmc message changes
    * [Pull request 36](https://bitbucket.org/osrf/srcsim/pull-requests/36)
    * [Issue 89](https://bitbucket.org/osrf/srcsim/issues/89/update-walking-script-to-use-new-ihmc)

* Random world generator
    * [Pull request 33](https://bitbucket.org/osrf/srcsim/pull-requests/33)
    * [Random world generator tutorial](https://bitbucket.org/osrf/srcsim/wiki/world_generator)

### SRC Sim 0.3.0 (2017-02-15)

* Random world generator
    * [Pull request 26](https://bitbucket.org/osrf/srcsim/pull-requests/26)
