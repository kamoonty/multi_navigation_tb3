/*
 * follower.cpp
 *
 *  Created on: Mar 31, 2014
 *      Author: roiyeho
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <algorithm>

#define MIN_DIST 0.5
#define MAX_LINEAR_VEL 0.2
#define MAX_ANGULAR_VEL 0.3

using namespace std;

int main(int argc, char** argv) {
	if (argc < 2) {
		ROS_ERROR("You must specify leader robot id.");
		return -1;
	}

	char *leader_id = argv[1];

	ros::init(argc, argv, "follower");
	ros::NodeHandle nh;

	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 0.15);
	tf::TransformListener listener;

	string tf_prefix;
	nh.getParam("tf_prefix", tf_prefix);
	string this_robot_frame = tf::resolve(tf_prefix, "base_footprint");
	cout << this_robot_frame << endl;

	string leader_str = "/tb3_";
	leader_str += leader_id;
	string leader_frame = tf::resolve(leader_str, "base_footprint");
	cout << leader_frame << endl;

	listener.waitForTransform(this_robot_frame, leader_frame, ros::Time(0), ros::Duration(10.0));

	ROS_INFO("%s is now following robot %s", tf_prefix.c_str(), leader_id);

	ros::Rate loopRate(10);
	while (ros::ok()) {
		tf::StampedTransform transform;

		try {
			listener.lookupTransform(this_robot_frame, leader_frame, ros::Time(0), transform);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
		}

		float dist_from_leader = sqrt(pow(transform.getOrigin().x(), 2) +
                pow(transform.getOrigin().y(), 2));


		geometry_msgs::Twist vel_msg;

		if (dist_from_leader > MIN_DIST) {
			vel_msg.linear.x = min(0.5 * dist_from_leader, MAX_LINEAR_VEL);
			vel_msg.angular.z = min(4 * atan2(transform.getOrigin().y(),
		                                  transform.getOrigin().x()), MAX_ANGULAR_VEL);
		}

		cmd_vel_pub.publish(vel_msg);
		loopRate.sleep();
	}

	return 0;
}

