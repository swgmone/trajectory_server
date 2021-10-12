# Trajectory Server
## An optimization-based trajectory planner
<img src="https://github.com/swgmone/trajectory_server/blob/main/images/traj.jpg" width = 47% height = 30%/><img src="https://github.com/swgmone/trajectory_server/blob/main/images/traj_field.svg" width = 43.8% height = 30%/>

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
