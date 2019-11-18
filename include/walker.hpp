/*
 * BSD License
 *
 * Copyright (c) Lydia Zoghbi 2019
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement
 * must be included in all copies of the Software, in whole or in part, and
 * all derivative works of the Software, unless such copies or derivative
 * works are solely in the form of machine-executable object code generated by
 * a source language processor.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *  @file       walker.hpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright BSD License
 *  @date       11/18/2019
 *  @version    1.0
 *
 *  @brief      Walker class header file
 *
 */
#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

/**
 *  @brief      A class for implemeting the walker 
 */
class Walker {
	public:

		/**
 		*  @brief      Constructor for Walker class
 		*  @param      None
 		*  @return     None
 		*/
		Walker();

		/**
 		*  @brief      Explore function for the robot
 		*  @param      None
 		*  @return     None
 		*/
		void Explore();

		/**
 		*  @brief      Function for setting the rate of publishing messages
 		*  @param      Constant rate integer
 		*  @return     Rate of message publishing
 		*/
		void SetRate(const int& rate);

		/**
 		*  @brief      Obtaining sensor readings from robot
 		*  @param      Laser scan sensor measurements
 		*  @return     None
 		*/
		void GetSensorReadings(const sensor_msgs::LaserScanConstPtr& scans);

		/**
 		*  @brief      Function for moving the robot straight
 		*  @param      The velocity at which the robot is to be moved
 		*  @return     Velocity (linear and angular) commands to the robot
 		*/
		geometry_msgs::Twist MoveStraight(const double& forwardVelocity);

		/**
 		*  @brief      Function for rotating the robot when needed
 		*  @param      Angle at which the robot is to be rotated
 		*  @return     Velocity (linear and angular) commands to the robot
 		*/
		geometry_msgs::Twist Rotate(const double& angle);

	private:
		// Create a node handle
		ros::NodeHandle nh;

		// Define a node publisher
		ros::Publisher pub;

		// Define a node subscriber
		ros::Subscriber sub;

		// Define velocity as geometry_msgs
		geometry_msgs::Twist velocity;

		// Define message rate type
		static int msg_rate;

		// Define angle type
		double angle;
};

#endif  // INCLUDE_WALKER_HPP_
