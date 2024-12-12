/**
 *  Created on: September 2019
 *  Author: jguerrero
 *  Example of odometry, laser and command vel topics
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "stdio.h"
#include "assert.h"
#include "math.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>

#include "nav_msgs/Odometry.h"

#include <string>
#include <sstream>
#include <iostream>

using namespace std;
using std::ostringstream;

struct Pose {
	double x;
	double y;
	double orientation;
};

ros::Publisher cmd_vel_pub; // publisher for movement commands
geometry_msgs::Point goal_point;
Pose current_pose;
ros::Time last_obstacle_time;

// constants from the launch file

double V_MAX_DES; // desired maximum velocity
double CRIT_DIST; // critical distance
double ORI_ERROR; // orientation error
double K_ROT_MIN; // minimum rotation
double K_ROT_MAX; // maximum rotation
double V_MAX_ROT; // maximum rotation velocity
double D_OBJ; // distance to object
int ALGOR; // algorithm

geometry_msgs::Twist initialize_cmd_vel() {
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0;
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;
	return cmd_vel;
}

double calculate_target_pose(double initial_x, double initial_y, double target_x, double target_y) {
	double dx = target_x - initial_x;
	double dy = target_y - initial_y;
	double pose = atan2(dy, dx);
	return pose;
}

void stop(geometry_msgs::Twist& cmd_vel) {
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z = 0.0;
}

void rotate(geometry_msgs::Twist& cmd_vel, double velocity = V_MAX_ROT) {
	cmd_vel.angular.z = velocity;
}

void move_forward(geometry_msgs::Twist& cmd_vel, double velocity = V_MAX_DES) {
	cmd_vel.linear.x = velocity;
}

bool is_facing_obstacle(const sensor_msgs::LaserScan& laser_data) {
	int length = laser_data.ranges.size();
	for(int i=0; i<length; i++) {
		if(laser_data.ranges[i] < CRIT_DIST) {
			return true;
		}
	}
	return false;
}

bool is_at_goal() {
	double distance_to_goal = sqrt(pow(goal_point.x - current_pose.x, 2) + pow(goal_point.y - current_pose.y, 2));
	return distance_to_goal <= D_OBJ;
}

void move_towards_goal(geometry_msgs::Twist& cmd_vel) {
	Pose target_pose;
	target_pose.x = goal_point.x;
	target_pose.y = goal_point.y;
	target_pose.orientation = calculate_target_pose(current_pose.x, current_pose.y, target_pose.x, target_pose.y);
	double angle_diff = target_pose.orientation - current_pose.orientation;
	// Take mod of the angle difference to get the minimum angle
	angle_diff = fmod(angle_diff, 2 * M_PI);
	// If the angle is more than PI, then the robot should turn from the other side
	if(angle_diff > M_PI) {
		angle_diff -= 2 * M_PI;
	} else if(angle_diff < -M_PI) {
		angle_diff += 2 * M_PI;
	}
	// If the angle difference is greater than the orientation error, turn
	if(fabs(angle_diff) < ORI_ERROR) {
		rotate(cmd_vel, K_ROT_MIN * V_MAX_ROT * angle_diff);
		move_forward(cmd_vel);
	} else {
		rotate(cmd_vel, K_ROT_MAX * V_MAX_ROT * angle_diff);
	}
}

void simple_avoidance(const sensor_msgs::LaserScan& laser_data) {	
    geometry_msgs::Twist cmd_vel = initialize_cmd_vel();
	// If the robot is at the goal, stop
	if(is_at_goal()) {
		ROS_INFO("At goal");
		stop(cmd_vel);
	}
	else {
		// ROS_INFO("Not at goal");
		// If there is an obstacle, stop and turn
		if(is_facing_obstacle(laser_data)) {
			// ROS_INFO("Obstacle detected");
			last_obstacle_time = ros::Time::now();
			stop(cmd_vel);
			rotate(cmd_vel);
		}
		else { // Path is clear
			//ROS_INFO("No obstacle detected");
			ros::Duration time_since_last_obstacle = ros::Time::now() - last_obstacle_time;
			// If there was an obstacle in the last 2 seconds, move forward
			if(time_since_last_obstacle.toSec() < 2.0) {
				// ROS_INFO("Obstacle detected recently, moving straight");
				stop(cmd_vel);
				move_forward(cmd_vel);
			}
			// Otherwise, move towards the goal
			else {
				//ROS_INFO("No obstacle detected recently, moving towards goal");
				stop(cmd_vel);
				move_towards_goal(cmd_vel);
			}
		}
	}
	cmd_vel_pub.publish(cmd_vel);
}


/**
  Read the the Laser data. If there is an obstacle close r the robot, stop and turn
  otherwise move ahead. Make the changes of the algorithm in this function.

  The callback laser should be the main function to handle the movement.
*/


void callback_laser(const sensor_msgs::LaserScan& most_intense) {
	// Display algorithm number
	if(ALGOR == 1) {
		simple_avoidance(most_intense);
	} else if(ALGOR == 2) {
		// Implement the second algorithm
	} else {
		// Raise an error
		ROS_ERROR("Invalid algorithm number");
	}
	ros::spinOnce(); 
}

/***
Get the odometry location.
***/
void callback_odom(const nav_msgs::Odometry odom) {

	double Xr = odom.pose.pose.position.x;
	double Yr = odom.pose.pose.position.y;
	double current_z=odom.pose.pose.orientation.z;
    double current_w=odom.pose.pose.orientation.w;
	double orientationr=(2.0*atan2(current_z,current_w));
	current_pose.x = Xr;
	current_pose.y = Yr;
	current_pose.orientation = orientationr;
}

/***
Get the goal location.
***/
void callback_goals(const geometry_msgs::Point goal) {
	goal_point = goal;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "moveRobot");
	ros::Time::init();

	ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

	nh.getParam("/moveRobot/V_MAX_DES", V_MAX_DES);
	nh.getParam("/moveRobot/CRIT_DIST", CRIT_DIST);
	nh.getParam("/moveRobot/ORI_ERROR", ORI_ERROR);
	nh.getParam("/moveRobot/K_ROT_MIN", K_ROT_MIN);
	nh.getParam("/moveRobot/K_ROT_MAX", K_ROT_MAX);
	nh.getParam("/moveRobot/V_MAX_ROT", V_MAX_ROT);
	nh.getParam("/moveRobot/D_OBJ", D_OBJ);
	nh.getParam("/moveRobot/ALGOR", ALGOR);

    //Build a string with the odom topoic
    string odom_topic_name = "robot_";
	odom_topic_name += "0";
	odom_topic_name += "/odom";
	// subscribe to robot's odom topic "robot_X/base_scan"
	ros::Subscriber odom_sub = nh.subscribe(odom_topic_name, 10, callback_odom);


    // subscribe to robot's laser scan topic "robot_X/base_scan"
	string sonar_scan_topic_name = "robot_";
	sonar_scan_topic_name += "0";
	sonar_scan_topic_name += "/base_scan_1";
	ros::Subscriber sub = nh.subscribe(sonar_scan_topic_name, 1, callback_laser);

	// subscribe to talker_goals topic
	ros::Subscriber goal_sub = nh.subscribe("myGoals", 1, callback_goals);

    string cmd_vel_topic_name = "robot_";
	cmd_vel_topic_name += "0";
	cmd_vel_topic_name += "/cmd_vel";
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

	ros::spin();
}

