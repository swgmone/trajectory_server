/* Trajectory Server v2                        */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: August 2020                           */
/* File: trajectory_server_node.cpp            */

#include <trajectory_server_v2/trajectory_server.hpp>

int main(int argc, char **argv){
  ros::init(argc, argv, "trajectory_server_node");
  ros::NodeHandle nh("~");

  TrajectoryServer trajServer(nh);
  ros::spin();

  return 0;
}