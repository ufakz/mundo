#include "ros/ros.h"
#include "geometry_msgs/Point.h"

double x_goal;
double y_goal;
double t_goal;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talkerGoals");
    ros::NodeHandle node_obj;
    ros::NodeHandle nh_private("~");
    nh_private.param<double>("X_GOAL", x_goal, 1.0);
    nh_private.param<double>("Y_GOAL", y_goal, 1.0);
    nh_private.param<double>("T_GOAL", t_goal, 1.0);
    ROS_INFO("X_GOAL=%lf", x_goal);
    ROS_INFO("Y_GOAL=%lf", y_goal);
    ROS_INFO("T_GOAL=%lf", t_goal);

    ros::Publisher goal_publisher = node_obj.advertise<geometry_msgs::Point>("myGoals", 10);
    ros::Rate loop_rate(1.0 / t_goal); 

    while (ros::ok())
    {
        geometry_msgs::Point point;
        point.x = x_goal;
        point.y = y_goal;
        point.z = 0.0;
        ROS_INFO("Goal: X=%f, Y=%f, Z=%f", point.x, point.y, point.z);
        goal_publisher.publish(point);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}