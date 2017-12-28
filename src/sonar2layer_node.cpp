#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "sonar_simulation/sonars.h"

#define MATH_PI 3.141592653589793

int sonar_number = 12;
std::vector<ros::Publisher> sonar_pubs;
float range_min = 0.2; //in meters
float range_max = 4; //in meters
float field_of_view = MATH_PI / 4.0; // in radian
float safe_range = 6; //in meters

void sonar_callback(const sonar_simulation::sonars& data)
{
	sensor_msgs::Range range;
	for (unsigned int k = 0; k < sonar_number; k++)
	{
		range.header.frame_id = data.sonar_list[k].frame_id;
		range.header.stamp = ros::Time::now();
		range.radiation_type = range.ULTRASOUND;
		range.field_of_view = field_of_view;
		range.min_range = range_min;
		range.max_range = range_max;
		if (data.sonar_list[k].range < safe_range)
		{
			if(0 == data.sonar_list[k].range) 
			{
				range.range = 0.5f;
			}else
			{
				float sonarRange = (float)(data.sonar_list[k].range);
				range.range = sonarRange*0.22f;
			}                                
		}
		else
		{
			range.range = 8.0f;
		} 
		sonar_pubs[k].publish(range);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sonar2layer_node");
	ros::NodeHandle n;
	ros::Subscriber sonar_sub = n.subscribe("sonars", 10, sonar_callback);

	std::stringstream index;
	for (unsigned int i = 0; i < sonar_number; i++)			// facing robot's back, 0-7 front sonar from left, 8-11 back sonar from left
	{
		
		index << "sonar_" << i;
                sonar_pubs.push_back(n.advertise<sensor_msgs::Range>(index.str(), 20));
		index.str("");
	}

        ros::Rate r(30);

	while (n.ok())
	{
		ros::spinOnce();
		r.sleep();
	}

	return 0;
	
}
