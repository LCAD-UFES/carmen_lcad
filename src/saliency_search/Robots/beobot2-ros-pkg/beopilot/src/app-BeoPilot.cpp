/*
 * Copyright (C) 2010, Chin-Kai Chang 
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "BeoPilot.H"
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sstream>

//joy -> joy/Joy.msg -> teleop -> geometry_msgs/Twist.msg -> this

BeoPilot * beopilot;
void cmd_vel_Callback(geometry_msgs::Twist msg)
{
	float trans = msg.linear.x;//Trans
	float rots = msg.angular.z;//Rot
	ROS_INFO("cmd_vel_callback: %f, %f", trans, rots);
	beopilot->SetMotorsPid(rots,trans);
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "beopilot");
  	ros::NodeHandle n;
  	ros::NodeHandle n_priv("~");
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/beobot2/odom", 1);
	tf::TransformBroadcaster odom_broadcaster;
	beopilot = new BeoPilot();
	beopilot->start();
	ros::Subscriber motor_sub = n.subscribe("/beobot2/cmd_vel",1,cmd_vel_Callback);

	ros::Rate pub_rate(10);
	double x = 0.0f;
	double y = 0.0f;
	double th = 0.0f;

	double vx = 0.0f;
	double vy = 0.0f;
	double vth = 0.0f;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	beopilot->resetEncoder();
	ROS_INFO("x %f y %f th %f vx %f vy %f vth %f", beopilot->itsPosition.x, beopilot->itsPosition.y, beopilot->itsPosition.z, beopilot->itsVelocity.x, beopilot->itsVelocity.y, beopilot->itsVelocity.z);
	
	while (ros::ok())
	{
		beopilot->UpdateRCStatus();
		x = -beopilot->itsPosition.x;
		y = beopilot->itsPosition.y;
		th = -beopilot->itsPosition.z;
		
		vx = -beopilot->itsVelocity.x;
		vy = beopilot->itsVelocity.y;
		vth = -beopilot->itsVelocity.z;
		
		ROS_INFO("x %f y %f th %f vx %f vy %f vth %f", x, y, th, vx, vy, vth);
		
		//update velocities here
		
		current_time = ros::Time::now();

		//compute odometry in a typical way given the velocities of the robot
		//double dt = (current_time - last_time).toSec();
		//double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		//double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		//double delta_th = vth * dt;

		//x += delta_x;
		//y += delta_y;
		//th += delta_th;

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "/beobot2/odom";
		odom_trans.child_frame_id = "/beobot2/base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0f;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0f;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		odom_pub.publish(odom);

		last_time = current_time;
		ros::spinOnce();
		pub_rate.sleep();
	}


	return 0;
}





