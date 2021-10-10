#include "ros/ros.h"
#include <trajectory_server/poly_lib.hpp>
#include <trajectory_server/PlanPolyTraj.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

void waypointsVisualization();
void prepareData();
void showTrajectory();

double wp_x[9] = {-11, -4, -4, 3, 3, 10, 10, 17, 17};
double wp_y[9] = {10, 10, -10, -10, 10, 10, -10, -10, 10};

std::vector<Polynomial*> traj_x, traj_y;
std::vector<double> trajTime;
trajectory_server::PlanPolyTraj srvMsg;
ros::Publisher marker_pub;

int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_client_test");
    ros::NodeHandle nh;

    ros::ServiceClient trajClient = nh.serviceClient<trajectory_server::PlanPolyTraj>("/plan_poly_trajectory");
    marker_pub = nh.advertise<visualization_msgs::Marker>("/traj", 1000);

    // Prepare Data
    prepareData();

    if(trajClient.call(srvMsg)){
        // Memorize Planned Trajectory
        traj_x.resize(srvMsg.response.times.size());
        traj_y.resize(srvMsg.response.times.size());

        // Memorize Start Times
        trajTime.resize(srvMsg.response.times.size() + 1);
        trajTime[0] = 0.0;

        for(int ii = 1; ii < trajTime.size(); ii++){
            trajTime[ii] = trajTime[ii - 1] + srvMsg.response.times[ii - 1];
        }

        int wpIndex = 0;
        for(int ii = 0; ii < traj_x.size(); ii++){
            std::vector<double> startConstraints_x, startConstraints_y, startConstraints_z,
                                endConstraints_x, endConstraints_y, endConstraints_z;

            startConstraints_x.push_back(srvMsg.response.wpoints[wpIndex].x);
            startConstraints_y.push_back(srvMsg.response.wpoints[wpIndex].y);

            endConstraints_x.push_back(srvMsg.response.wpoints[wpIndex + 1].x);
            endConstraints_y.push_back(srvMsg.response.wpoints[wpIndex + 1].y);

            for(int j = 0; j < srvMsg.response.wpoints[wpIndex].constraints.size(); j++){
                startConstraints_x.push_back(srvMsg.response.wpoints[wpIndex].constraints[j].x);
                startConstraints_y.push_back(srvMsg.response.wpoints[wpIndex].constraints[j].y);
            }

            for(int j = 0; j < srvMsg.response.wpoints[wpIndex + 1].constraints.size(); j++){
                endConstraints_x.push_back(srvMsg.response.wpoints[wpIndex + 1].constraints[j].x);
                endConstraints_y.push_back(srvMsg.response.wpoints[wpIndex + 1].constraints[j].y);
            }

            // Refresh WP Index
            wpIndex += 2;

            traj_x[ii] = new Polynomial(7, trajTime[ii], trajTime[ii + 1], startConstraints_x, endConstraints_x);
            traj_y[ii] = new Polynomial(7, trajTime[ii], trajTime[ii + 1], startConstraints_y, endConstraints_y);
        }

        ROS_INFO("Trajectory Generated Succesfully !!");
        showTrajectory();
    }
}

void prepareData(){
    ROS_INFO("Preparing data...");
    trajectory_server::Waypoint waypoint;
    trajectory_server::Constraint constraint, zeroConstraint;

    zeroConstraint.x = 0.0;
    zeroConstraint.y = 0.0;

    // Trajectory Should Start With actual Velocity and Position
    waypoint.x = wp_x[0];
    waypoint.y = wp_y[0];
    constraint.x = 0.0;
    constraint.y = 0.0;

    waypoint.constraints.insert(waypoint.constraints.end(), zeroConstraint);
    waypoint.constraints.insert(waypoint.constraints.end(), zeroConstraint);
    srvMsg.request.wpoints.push_back(waypoint);

    size_t n = sizeof(wp_x)/sizeof(wp_x[0]);
    for(int ii = 1; ii < n; ii++){
        waypoint.constraints.clear();
        waypoint.x = wp_x[ii];
        waypoint.y = wp_y[ii];

        if(ii == n-1){
            // Trajectory End Always With zero Velocity and Acceleration
            waypoint.constraints.insert(waypoint.constraints.end(), zeroConstraint);
            waypoint.constraints.insert(waypoint.constraints.end(), zeroConstraint);
        }
        
        srvMsg.request.wpoints.push_back(waypoint);
    }
}

void waypointsVisualization(){
    visualization_msgs::Marker waypoints;
    waypoints.header.frame_id = "map";
    waypoints.header.stamp = ros::Time::now();
    waypoints.ns = "traj";
    waypoints.pose.orientation.w = 1.0;
    waypoints.id = 0;
    waypoints.type = visualization_msgs::Marker::POINTS;
    waypoints.scale.x = 0.8;
    waypoints.scale.y = 0.8;
    waypoints.scale.z = 0.8;
    waypoints.color.g = 1.0f;
    waypoints.color.a = 1.0;

    waypoints.action = visualization_msgs::Marker::ADD;

    size_t n = sizeof(wp_x)/sizeof(wp_x[0]);
    for(int i = 0; i < n; i++){
        geometry_msgs::Point p;
        p.x = wp_x[i];
        p.y = wp_y[i];
        p.z = 0.0;
        waypoints.points.push_back(p);
    }

    marker_pub.publish(waypoints);
}

void showTrajectory(){
    ROS_INFO_STREAM("Show trajectory");
    visualization_msgs::Marker trajectory;
    waypointsVisualization();

    for(int i = 0; i < traj_x.size(); i++){
        for(double t = trajTime[i]; t < trajTime[i+1]; t += 0.01){
            geometry_msgs::Point p;
            p.x = traj_x[i]->evaluate(0, t);
            p.y = traj_y[i]->evaluate(0, t);
            p.z = 0.0;

            trajectory.points.push_back(p);
        }
    }
    
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = ros::Time::now();
    trajectory.ns = "traj";
    trajectory.action = visualization_msgs::Marker::ADD;
    trajectory.pose.orientation.w = 1.0;
    trajectory.id = 1;
    trajectory.type = visualization_msgs::Marker::POINTS;
    trajectory.scale.x = 0.1;
    trajectory.scale.y = 0.1;
    trajectory.scale.z = 0.1;
    trajectory.color.r = 1.0f;
    trajectory.color.a = 1.0;
    
    marker_pub.publish(trajectory);
}