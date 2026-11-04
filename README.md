\# TurtleBot3 Intruder Tracker



ROS (C++) node that makes a TurtleBot3 detect and follow a person ("intruder") using LiDAR, keeping a safe distance and using odometry to perform reorientation manoeuvres.



Final project of the Introduction to Robotics course, Bachelor's Degree in Artificial Intelligence, UPC.



\## Authors

\- Irene Miralles Ortega

\- Martí Andreu



\## How it works

The node `seguiment\_intrus` works as a state machine with 4 modes:



| Mode | Behaviour |

|------|-----------|

| 0 | Waiting for the start command |

| 1 | Following: the closest object in the front LiDAR sector (±22°) is the target. The robot keeps it between 0.5 and 0.6 m (advances or reverses) and turns slightly to centre it. It stops reversing if an obstacle is detected behind |

| 2 | 360° turn, measured by accumulating the yaw from odometry |

| 3 | 180° turn on manual command |



The robot enters mode 2 automatically after travelling 7 m in follow mode, or when a second start command is received.



\### Topics

| Topic | Type | Direction | Use |

|-------|------|-----------|-----|

| `scan` | `sensor\_msgs/LaserScan` | subscribes | Intruder detection |

| `odom` | `nav\_msgs/Odometry` | subscribes | Distance and angle travelled |

| `intruder\_command` | `std\_msgs/Bool` | subscribes | `true`: start following / trigger 360° turn |

| `manual\_turn` | `std\_msgs/Bool` | subscribes | `true`: trigger 180° turn |

| `cmd\_vel` | `geometry\_msgs/Twist` | publishes | Velocity commands |



\## Requirements

\- ROS (catkin) and the TurtleBot3 packages



Note: the `package.xml` of the original delivery is not included in this repository.



\## Build and run

```bash

cd \~/catkin\_ws/src

git clone https://github.com/irenemiralles/turtlebot3-intruder-tracker.git final\_im

cd \~/catkin\_ws \&\& catkin\_make

roslaunch final\_im final\_im.launch

```



Start tracking:

```bash

rostopic pub -1 /intruder\_command std\_msgs/Bool "data: true"

```



\## Report

Full project report: \[docs/report.pdf](docs/report.pdf)

