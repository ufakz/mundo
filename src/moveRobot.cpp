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

#define LEADER 0
#define FOLLOWER 1

using namespace std;
using std::ostringstream;

struct Pose {
	double x;
	double y;
	double orientation;
};

ros::Publisher cmd_vel_pub; // publisher for movement commands
geometry_msgs::Point goal_point; // global goal point
geometry_msgs::Point target_point; // target point for the robot (goal_point for leader, equal to leader odom for follower)
Pose current_pose;
ros::Time last_obstacle_time;
ros::Time last_decision_time; // for potential fields tracking

// constants from the launch file

double V_MAX_DES; // desired maximum velocity
double CRIT_DIST; // critical distance
double ORI_ERROR; // orientation error
double K_ROT_MIN; // minimum rotation
double K_ROT_MAX; // maximum rotation
double V_MAX_ROT; // maximum rotation velocity
double D_OBJ; // distance to object
double T_AVOID_OBS; // time to avoid obstacle
double W_1; // weight of go to target
double W_2; // weigth of avoid obstacles
int T_WAIT; // time to wait for potential fields algorithm
int ALGOR; // algorithm
int ID_ROBOT; // robot id
double DIST_LEADER; // distance from the follower to the leader
int ROBOT_ROL; // role of the robot
int ID_LEADER; // id of the leader

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

double calculate_distance(double x1, double y1, double x2, double y2) {
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

bool is_close_to_leader() {
	double distance_to_leader = calculate_distance(current_pose.x, current_pose.y, target_point.x, target_point.y);
	return distance_to_leader <= DIST_LEADER;
}

bool is_at_target() {
	// Find if the robot is at the target
	double distance_to_target = calculate_distance(target_point.x, target_point.y, current_pose.x, current_pose.y);
	return distance_to_target <= D_OBJ;	
}

bool is_leader_at_goal() {
	// Find if the leader is at the goal
	double distance_to_goal = calculate_distance(goal_point.x, goal_point.y, target_point.x, target_point.y);
	return distance_to_goal <= D_OBJ;
}

void move_towards_target(geometry_msgs::Twist& cmd_vel) {
	Pose target_pose;
	target_pose.x = target_point.x;
	target_pose.y = target_point.y;
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
		//TODO: Fix default time to use T_AVOID_OBS
		if(time_since_last_obstacle.toSec() < T_AVOID_OBS) {
			// ROS_INFO("Obstacle detected recently, moving straight");
			//stop(cmd_vel);
			move_forward(cmd_vel);
		}
		// Otherwise, move towards the target
		else {
			//ROS_INFO("No obstacle detected recently, moving towards target");
			//stop(cmd_vel);
			move_towards_target(cmd_vel);
		}
	}
	cmd_vel_pub.publish(cmd_vel);
}

void potential_fields_avoidance(const sensor_msgs::LaserScan& laser_data) {
    geometry_msgs::Twist cmd_vel = initialize_cmd_vel();

    // Get current time
    ros::Time current_time = ros::Time::now();

    // Check if enough time has passed since the last decision
    if ((current_time - last_decision_time).toSec() < (T_WAIT / 1000.0)) {
        return;
    }

    last_decision_time = current_time;
	
    // Calculate the attractive force to target
	double dx = target_point.x - current_pose.x;
    double dy = target_point.y - current_pose.y;
    double distance_to_target = sqrt(dx * dx + dy * dy);

    if(distance_to_target == 0) {
        ROS_WARN("Robot is exactly at the target position.");
        stop(cmd_vel);
        //cmd_vel_pub.publish(cmd_vel);
        return;
    }

    // Need to convert to unit vector to get magnitudes for each direction
    double Vobj_x = dx / distance_to_target;
    double Vobj_y = dy / distance_to_target;

	
	// ------Obstacle avoidance part------

	// Define accumulators
    double Vobs_x = 0.0;
    double Vobs_y = 0.0;

    // Loop through every laser scan reading
    for(size_t i = 0; i < laser_data.ranges.size(); i++) {
        double d_i = laser_data.ranges[i];

        if(d_i <= CRIT_DIST && d_i > 0) {
            
			// Get the angle of the current laser scan
            double angle = laser_data.angle_min + i * laser_data.angle_increment;

			double cos_angle = cos(angle);
			double sin_angle = sin(angle);

            double Xo_i = d_i * cos_angle;
            double Yo_i = d_i * sin_angle;
			// ROS_INFO("Obstacle position in robot frame: X=%f, Y=%f", Xo_i, Yo_i);

			// But the reading of the laser is in the robot reference frame
			// We need to convert to the world frame.
			
			// [x_world] = [cos θ  -sin θ] [x_robot] + [x_robot_position]
			// [y_world] = [sin θ   cos θ] [y_robot] + [y_robot_position]
			Xo_i = Xo_i * cos_angle - Yo_i * sin_angle + current_pose.x;
			Yo_i = Xo_i * sin_angle + Yo_i * cos_angle + current_pose.y;

			// ROS_INFO("Obstacle position in world frame: X=%f, Y=%f", Xo_i, Yo_i);

            double vec_x = Xo_i - current_pose.x;
			double vec_y = Yo_i - current_pose.y;

            // Calculate the magnitude of the repulsive force
            double magnitude = (CRIT_DIST - d_i) / CRIT_DIST;

            // Normalize the repulsive vector
            double norm = sqrt(vec_x * vec_x + vec_y * vec_y);
            if(norm == 0) {
                // Skip if the vector is zero to avoid division by zero
                continue;
            }
            double unit_x = vec_x / norm;
            double unit_y = vec_y / norm;

            // Apply the magnitude to the unit vector and accumulate
            Vobs_x += magnitude * unit_x;
            Vobs_y += magnitude * unit_y;
        }
    }

    // -------- Calculating the final Resultant Vector (Vf) --------

    double Vf_x = W_1 * Vobj_x + W_2 * Vobs_x;
    double Vf_y = W_1 * Vobj_y + W_2 * Vobs_y;

    double desired_orientation = atan2(Vf_y, Vf_x);

    double angle_diff = desired_orientation - current_pose.orientation;

    //angle_diff = fmod(angle_diff + M_PI, 2 * M_PI) - M_PI;

	angle_diff = fmod(angle_diff, 2 * M_PI);
	// If the angle is more than PI, then the robot should turn from the other side
	if(angle_diff > M_PI) {
		angle_diff -= 2 * M_PI;
	} else if(angle_diff < -M_PI) {
		angle_diff += 2 * M_PI;
	}

	// ROS_INFO("Angle difference: %f", angle_diff);

	// Setting the angular velocity

    // Proportional control for rotation
    double angular_z = V_MAX_ROT * K_ROT_MAX * angle_diff;

	// ROS_INFO("Angular Z: %f", angular_z);

    cmd_vel.angular.z = angular_z;

    // Setting the linear velocity
    if(abs(angle_diff) < ORI_ERROR) {
        cmd_vel.linear.x = V_MAX_DES;
    } else {
        cmd_vel.linear.x = 0.0;
    }

    cmd_vel_pub.publish(cmd_vel);
}


/**
  Read the the Laser data. If there is an obstacle close r the robot, stop and turn
  otherwise move ahead. Make the changes of the algorithm in this function.

  The callback laser should be the main function to handle the movement.
*/


void callback_laser(const sensor_msgs::LaserScan& most_intense) {
	// Common checks
	geometry_msgs::Twist cmd_vel = initialize_cmd_vel();
	
	// If the robot is the leader and is at the target, stop
	if(ROBOT_ROL == LEADER && is_at_target()) {
		// ROS_INFO("At target");
		stop(cmd_vel);
		cmd_vel_pub.publish(cmd_vel);
	}
	else if(ROBOT_ROL == FOLLOWER && is_leader_at_goal()) {
		// ROS_INFO("Leader is at goal");
		stop(cmd_vel);
		cmd_vel_pub.publish(cmd_vel);
	}
	else if(ROBOT_ROL == FOLLOWER && is_close_to_leader()) {
		// ROS_INFO("Leader is too close, stopping");
		stop(cmd_vel);
		cmd_vel_pub.publish(cmd_vel);
	}
	else {
		// Display algorithm number
		if(ALGOR == 1) {
			simple_avoidance(most_intense);
		} else if(ALGOR == 2) {
			// Implement the second algorithm
			potential_fields_avoidance(most_intense);
		} else {
			// Raise an error
			ROS_ERROR("Invalid algorithm number");
		}
	}
	ros::spinOnce(); 
}

/***
Get the odometry location.
***/

// Function to convert 3d quaternion to 2d orientation
Pose convert_odom_to_2d_orientation(const nav_msgs::Odometry& odom) {
	Pose pose;
	pose.x = odom.pose.pose.position.x;
	pose.y = odom.pose.pose.position.y;
	pose.orientation=(2.0*atan2(odom.pose.pose.orientation.z,odom.pose.pose.orientation.w));
	return pose;
}

void callback_odom(const nav_msgs::Odometry odom) {
	current_pose = convert_odom_to_2d_orientation(odom);
	ros::spinOnce(); 
}

void callback_leader_odom(const nav_msgs::Odometry odom) {
	Pose leader_pose = convert_odom_to_2d_orientation(odom);
	if(ROBOT_ROL == FOLLOWER) {
		target_point.x = leader_pose.x;
		target_point.y = leader_pose.y;
	}
	ros::spinOnce(); 
}

/***
Get the goal location.
***/
void callback_goals(const geometry_msgs::Point goal) {
	goal_point = goal;
	if(ROBOT_ROL == LEADER) {
		target_point = goal;
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "moveRobot");
	ros::Time::init();

	ros::NodeHandle node_obj;
	ros::NodeHandle nh_private("~");
	nh_private.param<int>("ID_ROBOT", ID_ROBOT, 1);
	nh_private.param<int>("ROBOT_ROL", ROBOT_ROL, LEADER);
	nh_private.param<int>("ID_LEADER", ID_LEADER, 0);
	nh_private.param<double>("DIST_LEADER", DIST_LEADER, 1.0);
	nh_private.param<double>("V_MAX_DES", V_MAX_DES, 1.0);
	nh_private.param<double>("CRIT_DIST", CRIT_DIST, 1.0);
	nh_private.param<double>("ORI_ERROR", ORI_ERROR, 0.1);
	nh_private.param<double>("K_ROT_MIN", K_ROT_MIN, 0.1);
	nh_private.param<double>("K_ROT_MAX", K_ROT_MAX, 1.0);
	nh_private.param<double>("V_MAX_ROT", V_MAX_ROT, 1.0);
	nh_private.param<double>("D_OBJ", D_OBJ, 0.5);
	nh_private.param<int>("ALGOR", ALGOR, 1);
	nh_private.param<double>("T_AVOID_OBS", T_AVOID_OBS, 2.0);
	nh_private.param<double>("W_1", W_1, 1.0);
	nh_private.param<double>("W_2", W_2, 1.0);
	nh_private.param<int>("T_WAIT", T_WAIT, 200);

	// Initialize last_decision_time to current time (once)
    last_decision_time = ros::Time::now();

    //Build a string with the odom topoic
    string odom_topic_name = "robot_";
	odom_topic_name += to_string(ID_ROBOT);
	odom_topic_name += "/odom";
	// subscribe to robot's odom topic "robot_X/base_scan"
	ros::Subscriber odom_sub = node_obj.subscribe(odom_topic_name, 10, callback_odom);

	//Build a string with the odom topic of the leader
	string leader_odom_topic_name = "/robot_";
	leader_odom_topic_name += to_string(ID_LEADER);
	leader_odom_topic_name += "/odom";
	// subscribe to robot's odom topic "robot_X/base_scan"
	ros::Subscriber leader_odom_sub = node_obj.subscribe(leader_odom_topic_name, 10, callback_leader_odom);

    // subscribe to robot's laser scan topic "robot_X/base_scan"
	string sonar_scan_topic_name = "robot_";
	sonar_scan_topic_name += to_string(ID_ROBOT);
	sonar_scan_topic_name += "/base_scan_1";
	ros::Subscriber sub = node_obj.subscribe(sonar_scan_topic_name, 1, callback_laser);

	// subscribe to talker_goals topic
	ros::Subscriber goal_sub = node_obj.subscribe("myGoals", 1, callback_goals);

    string cmd_vel_topic_name = "robot_";
	cmd_vel_topic_name += to_string(ID_ROBOT);
	cmd_vel_topic_name += "/cmd_vel";
	cmd_vel_pub = node_obj.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

	ros::spin();
}

