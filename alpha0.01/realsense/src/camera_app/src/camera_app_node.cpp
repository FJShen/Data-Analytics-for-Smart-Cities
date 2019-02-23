#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <sstream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <stdio.h>
#include <inttypes.h>
#include <vector>

int main(int argc, char** argv) {
  ros::init(argc, argv, "realsense");
  ros::NodeHandle rs;

  ros::Publisher depth_pub = rs.advertise<sensor_msgs::Image>("depth", 10);
  ros::Publisher rgb_pub = rs.advertise<sensor_msgs::Image>("RGB", 10);
  ros::Rate loop_rate(1);

  rs2::frameset frames;
  //rs2::depth_frame depth;
  
  rs2::pipeline p;
  p.start();

  while(ros::ok()) {
    sensor_msgs::Image depth_msg;
    sensor_msgs::Image rgb_msg;

    /*
      DEFINE SENSOR MESSAGE CONTENTS
    */
    
    std::cout<<"waiting for frames\n";
    frames = p.wait_for_frames();
    std::cout<<"frames received\n";
    rs2::depth_frame depth = frames.get_depth_frame();
   
    //rs2::video_frame color = frames.get_color_frame();

    unsigned int width = depth.get_width();
    unsigned int height = depth.get_height();
    const uint16_t* pixel16_ptr =(const uint16_t*) (depth.get_data());
    //const uint16_t* pixel16_ptr_color = (const uint16_t*) (color.get_data());
    const uint8_t* pixel_ptr = (const uint8_t*)(depth.get_data());
    //const uint8_t* pixel_ptr_color = (const uint8_t*)(color.get_data());
    //unsigned int width_color = color.get_width();
    //unsigned int height_color = color.get_height();
    unsigned int pixel_amount = width*height;
    //unsigned int pixel_amount_color = width_color*height_color;
	
    std::vector<uint8_t> depth_image(2*pixel_amount);
    //std::vector<uint8_t> color_image(2*pixel_amount);
    memcpy(&depth_image[0], pixel_ptr, 2*pixel_amount*sizeof(uint8_t));
    //memcpy(&color_image[0], pixel_ptr_color, 2*pixel_amount*sizeof(uint8_t));
    depth_msg.data = depth_image;
    depth_msg.height = height;
    depth_msg.width = width;
    depth_msg.encoding = sensor_msgs::image_encodings::MONO16;
    depth_msg.step = width*2; //each pixel is 2 bytes, so step, which stands for row length in bytes, should be two times "width".
	
    //rgb_msg.data = color_image;
    //rgb_msg.height = height;
    //rgb_msg.width = width;
    //rgb_msg.encoding = sensor_msgs::image_encodings::RGB16;
    //rgb_msg.step = width*2;   
    /*
      END DEFINE SENSOR MESSAGE CONTENTS
    */

	/*	uint8_t higher = depth_image[360*2*1280+640*2];
	uint8_t lower = depth_image[360*2*1280+640*2+1];
	uint16_t pixel = ((uint16_t)(higher)<<8) + (uint16_t)(lower);
	*/
    //std::cout<<pixel16_ptr[1280*360+640]<<"  ";
	//	std::cout<<pixel<<std::endl;

    depth_pub.publish(depth_msg);
    //rgb_pub.publish(rgb_msg);


    ros::spinOnce();

    loop_rate.sleep();
  }
}
