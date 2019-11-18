#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

class Walker {
	public:
		Walker();
		void Explore(const int& vel);
		void SetRate(const int& rate);
		geometry_msgs::Twist MoveStraight(const double& forwardVelocity);
		geometry_msgs::Twist Rotate(const double& angle);

	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_;
		static int msg_rate_;
		geometry_msgs::Twist velocity_;
};

