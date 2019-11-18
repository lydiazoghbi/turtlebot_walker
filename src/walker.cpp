#include "walker.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

int Walker::msg_rate_ = 1;

Walker::Walker() {
	pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
	velocity_.linear.x = 0;
	velocity_.linear.y = 0;
	velocity_.linear.z = 0;
	velocity_.angular.x = 0;
	velocity_.angular.y = 0;
	velocity_.angular.z = 0;
}

void Walker::SetRate(const int& rate) {
	msg_rate_ = rate;
}

geometry_msgs::Twist Walker::MoveStraight(const double& forwardVelocity) {
	velocity_.linear.x = forwardVelocity;
	velocity_.linear.y = forwardVelocity;
	return velocity_;
}

void Walker::Explore(const int& vel) {
	ros::Rate loop_rate(msg_rate_);

	while (ros::ok()) {
		pub_.publish(MoveStraight(1.5));
		ros::spinOnce();
		loop_rate.sleep();
	}
}

