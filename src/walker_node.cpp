#include <iostream>
#include "walker.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "walker_node");
	ros::NodeHandle nh("~");
	int rate;
	int stat = 0;
	nh.getParam("txstat", stat);
	
	Walker turtlebot;
	turtlebot.SetRate(rate);
	// int velocity = 5;
	if (stat == 1) {
		turtlebot.Explore();
	}
	return 0;
}
