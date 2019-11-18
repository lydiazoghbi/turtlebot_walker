#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>

class Walker {
	public:
		Walker();
		void Explore();
		void SetRate(const int& rate);
		void GetSensorReadings(const sensor_msgs::LaserScanConstPtr& scans);
		geometry_msgs::Twist MoveStraight(const double& forwardVelocity);
		geometry_msgs::Twist Rotate(const double& angle);

	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;
		geometry_msgs::Twist velocity;
		static int msg_rate;
		double angle;
};

