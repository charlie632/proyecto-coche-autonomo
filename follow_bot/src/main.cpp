/*
 * main.cpp
 *
 *  Created by viki on Nov 11, 2016
 *  Modified by Carbajal on Sep 25, 2018
 */

#include "Follower.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "follower");
	Follower follower;
	ros::spin();
	return 0;
}


