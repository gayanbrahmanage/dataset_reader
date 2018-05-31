#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
ros::Publisher pub_Laser;
ros::Publisher pub_Odom;
ros::Time currentTime;
void sendOdometryData(double x, double y, double theta);
void sendTransform();
int main(int argc, char **argv)
{
	std::ifstream laserfile("/home/thomas/intel_LASER_.txt");
	std::ifstream odofile("/home/thomas/intel_ODO.txt");
	if(laserfile.is_open() && odofile.is_open())
	{
		ros::init(argc, argv, "Reader_Node");
		ros::NodeHandle n;
		pub_Laser = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
		pub_Odom = n.advertise<nav_msgs::Odometry>("odom", 1000);
		ros::Rate loop_rate(20);
		std::string line;
		int seq =0;
		double currentx=0, currenty=0, currenttheta=0;
		while(std::getline(laserfile, line)&&ros::ok())
		{
			currentTime = ros::Time::now();

			std::istringstream laseriss(line);
			int num_of_readings =0 ;
			sensor_msgs::LaserScan msg;
			msg.header.seq = seq++;
			msg.header.stamp = currentTime;
			msg.header.frame_id  = "scan";
			float range;
			while (laseriss>>range)
			{
				msg.ranges.push_back(range);
				num_of_readings++;
			}
			msg.angle_min = -2.35619; //starting_angle 
			msg.angle_max = 2.35619 ;
			msg.angle_increment = 4.71239/num_of_readings;
			msg.range_min = 0;
			msg.range_max = 80;
			std::getline(odofile, line);
			std::istringstream odoiss(line);
			double x, y, theta;
			odoiss>>x>>y>>theta;
			currentx+=x;
			currenty+=y;
			pub_Laser.publish(msg);
			sendTransform();
			sendOdometryData(currentx,currenty,theta);
			ros::spinOnce();
			loop_rate.sleep();
		}
			
	}
	else
	{
		std::cout<<"could not open the file"<<std::endl;
	}
	return 0;
}
void sendOdometryData(double x, double y, double theta)
{
	static tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = currentTime;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);
	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = "odom";
	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

	//set the velocity

	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = 0;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = 0;

	//publish the message
	pub_Odom.publish(odom);	
}
void sendTransform ()
{
	static tf::TransformBroadcaster broadcaster_;
	broadcaster_.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //x,y,z (0.05, 0.0, 0.75)
	  currentTime, "base_link", "scan" ));
}
