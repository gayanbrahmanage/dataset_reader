#include "reader.h"
/* This program reads a raw log file and publishes the scanner data and the odometry data for gmap.
	Current works for intel dataset rawlog 2003
   Author: Thomas Vy
   Email: thomas.vy@ucalgary.ca
   Date: May 31, 2018
   Version: 1.0
*/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Reader_Node");
	ros::NodeHandle n;  
	pub_Laser = n.advertise<sensor_msgs::LaserScan>("scan", 1000); //publishes to the topic 'scan'.
	pub_Odom = n.advertise<nav_msgs::Odometry>("odom", 1000); //publishes to the topic 'odom'.
	start("/home/thomas/catkin_ws/src/dataset_reader/src/intel.log"); //the location(full path) of the raw log Ex. /home/thomas/catkin_Ws/src/dataset_reader/src/intel.log
	return 0;
}