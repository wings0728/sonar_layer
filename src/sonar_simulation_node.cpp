#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sonar_simulation/sonars.h"

int sonar_num = 12;
unsigned char sonarsBuf[sonar_num] = {0};
void sonarsCallback(const sensor_msgs::Imu& sonars_msg)
{
	unsigned char idx_ = 0;
	for(idx_ = 0; idx_ < 8; idx_ ++)
	{
		sonarsBuf[idx_] = sonars_msg.orientation_covariance[idx_];
	}
	for(idx_ = 0; idx_ < 4; idx_ ++)
	{
		sonarsBuf[idx_ + 8] = sonars_msg.angular_velocity_covariance[idx_];
	}
//	ROS_INFO("in");

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sonar_simulation_node");
	ros::NodeHandle n;
	ros::Subscriber sonar_sub = n.subscribe("sonar_msg", 1000, sonarsCallback);
	ros::Publisher sonar_pub = n.advertise<sonar_simulation::sonars>("sonars", 1000);
//ROS_INFO("begin");
//	float front_sonar_value = 0;
//	bool increase_flag = true; // true - front sonar value increases, false - front sonar value decreases

	sonar_simulation::sonars snr;
	snr.header.frame_id = "sonars";

	std::vector<sonar_simulation::sonar> ss;
	std::string sonar_id[sonar_num] = {"lower_front1_left_sonar", "upper_front1_left_sonar", "lower_front2_left_sonar",\
											"main_left_sonar", "main_right_sonar", "lower_front2_right_sonar",\
											"upper_front1_right_sonar", "lower_front1_right_sonar", "back1_left_sonar",\
											"back2_left_sonar", "back2_right_sonar", "back1_right_sonar"};

	for (unsigned int i = 0; i < sonar_num; i++)
	{
		sonar_simulation::sonar ss_temp;
		ss_temp.frame_id = sonar_id[i];
		ss.push_back(ss_temp);
		
	}

	
	ros::Rate r(10); // set the loop rate as 1Hz

	while (n.ok())
	{

		snr.header.stamp = ros::Time::now();
		for(unsigned char idx_ = 0; idx_ < sonar_num; idx_ ++)
		{
			ss[idx_].range = sonarsBuf[idx_];
		}
/*
		for (unsigned int j = 0; j < sonar_num-4; j++)
		{
			ss[j].range = front_sonar_value;	
		}
*/		

		snr.sonar_list = ss;
		sonar_pub.publish(snr);
/*
		if (increase_flag)
			front_sonar_value += 0.1;
		else front_sonar_value -= 0.1;
		
		if (front_sonar_value < 0.1) increase_flag = true; // if the sonar value is 0, increase the sonar value
		else if (front_sonar_value > 3) increase_flag = false; // if the sonar value is 8, decrease the sonar value
*/		
		ros::spinOnce();
		r.sleep();

	}

	return 0;
}
