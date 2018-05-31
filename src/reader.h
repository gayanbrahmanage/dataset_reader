#ifndef READER
#define READER
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <fstream>
#include <string>
#include <sstream>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
/* This program reads a raw log file and publishes the scanner data and the odometry data for gmap.
	Current works for intel dataset rawlog 2003
   Author: Thomas Vy
   Email: thomas.vy@ucalgary.ca
   Date: May 31, 2018
   Version: 1.0
*/
ros::Publisher pub_Laser; //The publisher for the laser data
ros::Publisher pub_Odom; //The publisher for the odometry data
ros::Time currentTime; //The time the data is published
void sendOdometryData(double x, double y, double theta); //Sends the Odometry data 
void sendTransform(); //Sends the transform of the scanner with respect to the base_link
//Starts the map drawing
void start(std::string name)
{
	std::ifstream file(name.c_str()); //loads the raw log file.

	if(file.is_open()) //Checks if the file can be opened
	{
		ros::Rate loop_rate(30); //The rate at which it pushes the data. The slower the publishing, the more accurate the map.
		std::string line; //The current line of the file.
		int seq =0; //The sequence number for the laser message.
		while(std::getline(file, line) && ros::ok()) //reads the line of the file and checks if the the file is at the end or the program is killed.
		{
			currentTime = ros::Time::now(); //Gets the current time
			std::stringstream ss(line); //turns the line into a string stream
			std::string firstWord; //first word of the line
			ss>>firstWord;
			if(firstWord == "FLASER") //Checks if the line is laser data
			{
				sensor_msgs::LaserScan msg; //The laserscan message
				msg.header.seq = seq++;
				msg.header.stamp = currentTime;
				msg.header.frame_id  = "scan"; //Laserscan message on the scan topic

//////////////////////////////////////////////////////////////////////////////////////////
				//Scanner and file specifics
				int num_of_readings; //Number of points projected
				ss>>num_of_readings;
				msg.angle_min = -1.5708; //starting_angle of the scanner in radians 
				msg.angle_max = 1.5708 ; //ending angle of the scanner in radians
				msg.angle_increment = 3.14159/num_of_readings; //The angle increment per dot
				msg.range_min = 0; //The minimum range of the scanner
				msg.range_max = 80; //The maximum range of the scanner

				for(int i=0; i<num_of_readings;i++) //Reads the dot ranges and places them in the laserscan message
				{
					float range;
					ss>>range;
					msg.ranges.push_back(range);
				}
//////////////////////////////////////////////////////////////////////////////////////////

				pub_Laser.publish(msg); //publishes laserscan message
				sendTransform();
				ros::spinOnce();
				loop_rate.sleep();

			}
			else if (firstWord == "ODOM") //Checks if the line is odometry data
			{
				double x, y, theta; //The pose of the robot
				ss>>x;
				ss>>y;
				ss>>theta;
				sendOdometryData(x,y,theta);
				sendTransform();
				ros::spinOnce();
				loop_rate.sleep();
			}
				
		}
			
	}
	else //The file cannot be opened
	{
		std::cout<<"could not open the file"<<std::endl;
	}
}
//sends odometry data
void sendOdometryData(double x, double y, double theta) 
{
	static tf::TransformBroadcaster odom_broadcaster; //The broadcaster for the odometry ->baselink transform.
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = currentTime;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0; //No transformation in the z direction. Strictly in the x,y direction
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);

	odom_broadcaster.sendTransform(odom_trans);




	//publishing the odometry message
	nav_msgs::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = "odom";
	//position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

	//velocity

	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = 0;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = 0;

	pub_Odom.publish(odom);	
}
void sendTransform ()
{
	static tf::TransformBroadcaster broadcaster_; //The broadcaster for the baselink -> scan transform
	broadcaster_.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //Says the scan is the same position as the baselink.
	  currentTime, "base_link", "scan" ));
}
#endif