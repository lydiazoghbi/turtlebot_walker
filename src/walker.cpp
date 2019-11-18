#include "walker.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>

int Walker::msg_rate = 1;

Walker::Walker() {
	pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
	sub = nh.subscribe("/scan", 500, &Walker::GetSensorReadings, this);
	velocity.linear.x = 0;
	velocity.linear.y = 0;
	velocity.linear.z = 0;
	velocity.angular.x = 0;
	velocity.angular.y = 0;
	velocity.angular.z = 0;
}

void Walker::SetRate(const int& rate) {
	msg_rate = rate;
}

geometry_msgs::Twist Walker::MoveStraight(const double& forwardVelocity) {
	velocity.linear.x = forwardVelocity;
	velocity.angular.z = 0;
	return velocity;
}

geometry_msgs::Twist Walker::Rotate(const double& angle) {
	velocity.linear.x = 0.2;
	velocity.angular.z = angle;
	return velocity;
}

void Walker::GetSensorReadings(const sensor_msgs::LaserScanConstPtr& scans) {
	auto minimumDistance = std::min_element(scans -> ranges.begin(), scans -> ranges.end());
	double threshold = 4.0;
	if(*minimumDistance <threshold) {
		auto iterations = std::find(scans->ranges.begin(), scans->ranges.end(), *minimumDistance);
		auto currentPosition = iterations - scans->ranges.begin();
		double absDistance = scans->ranges.end() - scans->ranges.begin();

		if (currentPosition <= (absDistance/2.0)) {
			angle = 1;
		} else if (currentPosition >  (absDistance/2.0)) {
			angle = -1;
		}
	} else {
		angle = 0;
	}
}

void Walker::Explore() {
	ros::Rate loop_rate(msg_rate);
	pub.publish(MoveStraight(0.1));

	while (ros::ok()) {
		ros::Rate loop_rate(msg_rate);
		if (angle == 0) {
			pub.publish(MoveStraight(0.3));
		}
		
		while (angle == 1) {
			pub.publish(Rotate(-1));
			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO_STREAM("Rotating clockwise");
		}
		
		while (angle == -1) {
			pub.publish(Rotate(1));
			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO_STREAM("Rotating counter-clockwise");
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

