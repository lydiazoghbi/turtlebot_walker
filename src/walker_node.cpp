#include <iostream>
#include "walker.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "walker_node");
	ros::NodeHandle nh("~");
	int rate;
	int stat = 0;
	nh.getParam("txstat", stat);
	
	Walker turtlebot;
	turtlebot.SetRate(rate);
	int velocity = 5;
	if (stat == 1) {
		turtlebot.Explore(rate);
	}
	return 0;
}
