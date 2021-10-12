# Trajectory Server
## An Optimization-based Trajectory Planner
<img src="https://github.com/swgmone/trajectory_server/blob/main/images/traj.jpg" width = 47% height = 30%/><img src="https://github.com/swgmone/trajectory_server/blob/main/images/traj_field.svg" width = 43.8% height = 30%/>

The trajectory_server module is an optimization-based trajectory planner able to process an unlimited number of waypoints, each of which specified as:
1) *w<sub>i</sub>* the plane position of the waypoint,
2) *C<sub>i</sub>*  a vector of constraints.

The implemented optimization procedure allows to generate optimal polynomial trajectories by weighting the overall travel time and the trajectory smoothness.
The flexibility of our implementation allows to specify a variable number of derivative constraints for each waypoints letting the remaining quantities, needed to completely define the polynomial curve, to be free. These degrees of freedom are used by the trajectory optimization procedure to minimize a fixed trajectory cost.

## Authors
  * Lorenzo Gentilini - PhD Student
    * Email: lorenzo.gentilini6@unibo.it
  * Simone Rossi - PhD Student
    * Email: simone.rossi39@unibo.it
  * Dario Mengoli - PhD Student
    * Email: dario.mengoli2@unibo.it
  * Andrea Eusebi - Research Fellow
    * Email: andrea.eusebi5@unibo.it
  * Lorenzo Marconi - Full Professor
    * Email: lorenzo.marconi@unibo.it


## Reference
If you make use of this code, please cite:
* L. Gentilini, S. Rossi, D. Mengoli, A. Eusebi, L. Marconi. **Trajectory Planning ROS Service for an Autonomous Agricultural Robot**. IEEE MetroAgriFor 2021. ([Paper](--), [BibTex](--)).

## Trajectory Generation Example
In order to run this example, please runs the following commands:
1) Set-up your ROS environment
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
2) Clone the *trajectory_server* repository and build the project
```
git clone git@github.com:swgmone/trajectory_server.git
cd ~/catkin_ws
catkin_make
```
3) Launch the *trajectory_server* module
```
roslaunch trajectory_server trajectory_server_node
```
4) Run the client
```
rosrun trajectory_server trajectory_test_node
```
The provided example generate an optimal polynomial trajectory passing through nine waypoints:
```
x1 = -11.0, y1 = 10.0
x2 = -4.0,  y2 = 10.0
x3 = -4.0,  y3 = -10.0
x4 = 3.0,   y4 = -10.0
x5 = 3.0,   y5 = 10.0
x6 = 10.0,  y6 = 10.0
x7 = 10.0,  y7 = -10.0
x8 = 17.0,  y8 = -10.0
x9 = 17.0,  y9 = 10.0
```