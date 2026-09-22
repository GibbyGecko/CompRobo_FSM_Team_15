# CompRobo_FSM_Team_15
# RoboBehaviors and Finite State Machines Project

Author names: Ophelia Lonzo, Liam Brennan

## Project Overview

For this project we programmed a Neato to run a sequence of behaviors inside a finite state machine. During this sequence it drives a square, drives forward in a spiral until it; sees something or you push both side bumpers, and then finds a wall and follows it. During this sequence a bump-sensor e-stop is running for safety. 


Key design choices:
- Only one node controls`cmd_vel`. This node calls each behavior node, cancels their timers, and calls just the active behavior each tick, so there is no conflict over which behaviors is getting published to `cmd_vel`.
- The emergency stop is not a state, but a override that is constantly running. 
- The turning during the driving in square state uses odometry yaw instead of timing, after timed turns caused angle inaccuracies.



https://youtu.be/z4P78htCtCk
## Individual Behaviors

### Behavior 1: Drive square (`drive_square.py`)

**What it does.** The Neato drives a 1 m by 1 m square: straight for 1 m, turn 90 degrees left, repeated four times, and then stops.

**Implementation.** `DriveSquareNode` runs a 10 Hz timer that publishes `geometry_msgs/Twist` to `cmd_vel`. It is small state machine (straight, turn) with a counter of completed sides. It subscribes to `odom` (`nav_msgs/Odometry`) and converts its orientation to an angle.

**Design decisions.** Straight segments are timed (length of a side divided by speed, 5 s at 0.2 m/s). Our first version also timed the turns (90 degrees divided by the angular speed), but the turns were very inaccurate as the neato is generally inaccurate. The node now records the yaw at the start of the turn and turns until the change in yaw reaches pi/2,, slowing down to avoid overshoot. 

<img width="578" height="796" alt="Screencast from 2026-09-20 22-00-58" src="https://github.com/user-attachments/assets/3e37232f-ab2c-41c4-8b62-89f857bd338a" />

Bag: `bags/drive_square_demo`

### Behavior 2: Emergency stop (`bump_estop.py`)

**What it does.** Stops the robot when any of the fbump sensors is triggered.

**Implementation.** `EmergencyStopNode` subscribes to `bump` (`neato2_interfaces/Bump`) and keeps a `bump_state` flag.

**Design decisions.** The stop does not end the state machine, the robot resumes when the bumpers release. On its own the node also drives forward at 0.1 m/s when nothing is bumped, so inside the FSM we do not run its timer and only read `bump_state`.

### Behavior 3: Collision avoidance (`collision_avoidance.py`)

**What it does.** Drives forward slowly and stops when something is within 1 m in front.

**Implementation.** `CollisionAvoidanceNode` subscribes to `scan` (`sensor_msgs/LaserScan`) and reads the ray straight ahead, and publishes 0.1 m/s forward or zero to 'cmd_vel`.

**Design decisions.** When run by itself it is able to switch back and forth between the stopped and driving states so it can continue if the obstacle moves out of the way, however when being run with the rest of our FSM the state change to stopped acts as a trigger to switch to the next behavior.

<img width="578" height="796" alt="Screencast from 2026-09-20 22-15-10" src="https://github.com/user-attachments/assets/195dcb5b-edf9-4aa7-a610-4723ece6a0b7" />

Bag: `bags/collision_avoidance_demo`

### Behavior 4: Spiral Collision avoidance (`spiral_collision_avoidance.py`)

**What it does.** Drives in a expanding spiral until if it sees something in a 30 Deg cone in front of it within 1 m.

**Implementation.** `SpiralCollisionAvoidanceNode` subscribes to `scan` (`sensor_msgs/LaserScan`) and reads the rays in a 30 Deg cone in front of itself, and publishes its linear and angular velocities to 'cmd_vel' such that it follows a spiral that starts with a radius of 0.2 m which grows by 0.03 m every 0.1 sec.

**Design decisions.** 

<img width="578" height="796" alt="Screencast from 2026-09-20 22-50-39" src="https://github.com/user-attachments/assets/5bbef3ca-e2b9-4869-ae3c-09619e4af2cb" />

Bag: `bags/spiral_collision_avoidance`

### Behavior 4: Wall following (`wall_follower.py`)

**What it does.** Approaches a wall, turns until it is parallel, then drives along it at a set distance.

**Geometry.** When the wall is on the robot's right the node uses two laser rays 30 degrees either side of the perpendicular (at angle 300 and 240). If the front ray is longer than the back ray, the robot is pointing away from the wall. The angle error is the difference between the two ranges, and the distance to the wall is their mean times cos(30 degrees).

**Implementation.** `WallFollowingNode` subscribes to `scan` and publishes `Twist` to`cmd_vel` . It has three states:
- Approach: proportionally approches until it is 0.5 m from the wall.
- Turn: rotates in place until the two side rays are within 0.05 m of each other.
- Follow: drive at 0.3 m/s with a proportional angular velocity to the angle error and the distance error.

**Design decisions.** 


**Visualization.** The detected wall is drawn in rviz as a line through the two ray hit points.(the two gifs are not of the same simulation)

<img width="790" height="682" alt="Screencast from 2026-09-20 23-12-20" src="https://github.com/user-attachments/assets/ba63bdb4-112d-45fa-83c6-3aa2606b455b" />


<img width="578" height="796" alt="Screencast from 2026-09-20 22-33-05" src="https://github.com/user-attachments/assets/b391bff8-67a6-472b-9ac3-aab1470d04be" />

Bag: `bags/wall_follower_demo`

## Finite State Machine

### Overall Design

A Neato running the FSM drives a square, drives forward in a spiral until it; sees a wall to follow or both side bumpers are pressed. When both side bumpers are pressed (or Enter is pressed for simulation), it approaches the wall, turns parallel, and follows it. At any point, pressing any bumper e-stops the robot until the bumper is released.

States:
- Drive square: runs `drive_square.py` until four sides are completed.
- Spiral collision avoidance: runs `spiral_collision_avoidance.py`, which drives in an expanding spiral with lidar collision avoidance.
- Wall following: runs `wall_follower.py` (approaches, turns, follows).

Transitions:
- Drive square to spiral collision avoidance: four sides completed.
- Spiral collision avoidance to wall following: both side bumpers pressed, or Enter in simulation.
- Emergency stop: any bump publishes zero velocity in any state.

<img width="481" height="253" alt="Untitled" src="https://github.com/user-attachments/assets/a2eee690-da7d-4a56-84b7-e68211983980" />


[FSM diagram]
### Implementation Details

`finite_state_controller.py` contains `BehaviorFSMNode`. `main()` creates one instance of each node and adds them all to a `SingleThreadedExecutor`, so every node keeps receiving its sensor data. The FSM cancels each behavior's own timer, then runs a 10 Hz timer of its own that calls the current behavior's `run_loop()`. Because only one `run_loop()` runs at a time, only one node ever publishes to `cmd_vel`.

Each tick, the FSM first checks the collision-avoidance transition (both side bumpers, or the keyboard flag), then the emergency stop via `EmergencyStopNode.bump_state`, and only then runs the current behavior. Progress through the square is read from the node's own `sides_completed` counter.

We chose this over rewriting all behaviors in one node so each file stays a standalone, testable node with its own `main()'.
This also made creating the FSM much easier and made tuning each behavior much more convenient

Capabilities and limitations:
- The FSM is a fixed sequence with no transitions back.
- Wall following never exits.
- The robot must start facing a wall more than about 1.5 m away.


### Demonstration

When running our code on a real neato(EX: https://youtu.be/Rbukj9MPeXE?si=hfSjVmLXsk7lxJlQ) the only change we had to make was slightly modifying the angles of the corners in the square drawing node to account for the slight difference between the real and simulation neato.

Bag: `bags/finite_state_controller_demo`

## Challenges

- Turns overshot 90 degrees, we switched to use odometry .
- Behaviors written as standalone nodes all publish to `cmd_vel`, so running them together made them fight. The supervisor design fixed this.
- `bump_estop.py` drives the robot when nothing is bumped, so it could not run alongside other behaviors as written.

## Future Improvements

- A emergency stop that needs a reset.
- More robust wall detection using many rays instead of two.
- A transition that does not depend on a person pressing the bumpers.


## Learning Objectives and Final Takeaways

- Learn how to use ros2.
- Learn how to effectively use a simulator to test code.
- Odometrey is very useful. Timing a turn assumes the robot does exactly what it is told; odometry checks it.
- Plan for how to manage which node gets `cmd_vel`. 
- Test on the real robot early. 

## How To Run

```bash
cd ~/ros2_ws/src
git clone [https://github.com/GibbyGecko/CompRobo_FSM_Team_15] ros_behaviors_fsm
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
```

Simulator: `ros2 launch neato2_gazebo neato_gauntlet_world.py`



Play a bag (disconnect from the robot first): `ros2 bag play bags/<name> --clock`, then open rviz with the saved config.
