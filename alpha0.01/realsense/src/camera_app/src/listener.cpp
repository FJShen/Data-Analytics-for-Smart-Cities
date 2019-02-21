#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <stdio.h>
#include <inttypes.h>
#include <vector>

void chatterCallback(const sensor_msgs::Image imsg)
{
	//uint32_t recv_height = (imsg->height);
	//uint32_t recv_width = (imsg->width);
	//uint32_t index = (0.5*height*width+0.5*width)/2*2;
	//uint16_t upper_pixel, lower_pixel;
	//upper_pixel = imsg->data[index];
	//lower_pixel = imsg->data[index+1];
	//uint16_t pixel = (uint16_t)((upper_pixel<<8)|lower_pixel);
	//ROS_INFO("height, width, middle pixel= %u, %u\n", height, width);
	//ROS_INFO("recieved\n");
	std::cout<<"received\n";
	//std::cout<<"h*w="<<recv_height<<","<<recv_width<<std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("depth", 10, chatterCallback);
  std::cout<<"this is listener!\n";
  ros::spin();
}
