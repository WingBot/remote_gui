/*
 * move_robot.cpp
 *
 *  Created on: Mar 9, 2014
 *      Author: roiyeho
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include <string>
using namespace std;

#define MAX_ROBOTS_NUM 20
#define FORWARD_SPEED 0.2
#define MIN_SCAN_ANGLE -60.0/180*M_PI
#define MAX_SCAN_ANGLE +60.0/180*M_PI
#define MIN_PROXIMITY_RANGE 0.5

int robot_id;
ros::Publisher cmd_vel_pub; // publisher for movement commands
ros::Subscriber laser_scan_sub; // subscriber to the robot's laser scan topic
bool keepMoving = true;

void move_forward();
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

int main(int argc, char **argv)
{
	if (argc < 2) {
		ROS_ERROR("You must specify robot id.");
		return -1;
	}

	char *id = argv[1];
	robot_id = atoi(id);

	// Check that robot id is between 0 and MAX_ROBOTS_NUM
	if (robot_id > MAX_ROBOTS_NUM || robot_id < 0 ) {
	    ROS_ERROR("The robot's ID must be an integer number between 0 an 19");
	    return -1;
	}
	ROS_INFO("moving robot no. %d", robot_id);

	// Create a unique node name
	string node_name = "move_robot_";
	node_name += id;
	cout << node_name;

	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	// cmd_vel_topic = "robot_X/cmd_vel"
	string cmd_vel_topic_name = "robot_";
	cmd_vel_topic_name += id;
	cmd_vel_topic_name += "/cmd_vel";
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

	// subscribe to robot's laser scan topic "robot_X/base_scan"
	string laser_scan_topic_name = "robot_";
	laser_scan_topic_name += id;
	laser_scan_topic_name += "/base_scan";
	laser_scan_sub = nh.subscribe(laser_scan_topic_name, 1, &scanCallback);

	move_forward();
	return 0;
}

void move_forward()
{
	// Drive forward at a given speed.
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = FORWARD_SPEED;
	cmd_vel.angular.z = 0.0;

	// Loop at 10Hz, publishing movement commands until we shut down
	ros::Rate rate(10);

	while (ros::ok() && keepMoving) // Keep spinning loop until user presses Ctrl+C
	{
		cmd_vel_pub.publish(cmd_vel);
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		rate.sleep();
	}

	// Stop the robot
	geometry_msgs::Twist stop_cmd_vel;
	stop_cmd_vel.linear.x = 0.0;
	stop_cmd_vel.angular.z = 0.0;
	cmd_vel_pub.publish(stop_cmd_vel);

	ROS_INFO("robot no. %d stopped", robot_id);
}

// Process the incoming laser scan message
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// Find the closest range between the defined minimum and maximum angles
	int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

	float closestRange = scan->ranges[minIndex];
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
		if (scan->ranges[currIndex] < closestRange) {
			closestRange = scan->ranges[currIndex];
		}
	}

	//ROS_INFO_STREAM("Closest range: " << closestRange);

	if (closestRange < MIN_PROXIMITY_RANGE) {
		keepMoving = false;
	}
}
