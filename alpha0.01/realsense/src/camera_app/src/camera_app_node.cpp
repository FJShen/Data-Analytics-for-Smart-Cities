#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <sstream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <stdio.h>
#include <inttypes.h>
#include <vector>
#include <chrono>

#define FPS 60 //{15, 30, 60, 90}
//#define PRNT_T_STMP //controls printing timestamps to screen or not

inline void print_time_stamp(std::string comment="--");

int main(int argc, char** argv) {
  ros::init(argc, argv, "realsense");
  ros::NodeHandle rs;

  ros::Publisher depth_pub = rs.advertise<sensor_msgs::Image>("depth", 8);
  ros::Publisher rgb_pub = rs.advertise<sensor_msgs::Image>("RGB", 8);
  ros::Rate loop_rate(FPS);

  rs2::frameset frames;
  //rs2::depth_frame depth;
  
  rs2::pipeline p;
  rs2::config c;
  c.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, FPS);
  c.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, (FPS>60)?60:FPS);//according to Intel RS documentation, the max FPS of rgb stream is 60 
  p.start(c);

  uint32_t seq = 0;
  while(ros::ok()) {
    std::cout << "Seq: " << seq << "\t";
    print_time_stamp("start");
    sensor_msgs::Image depth_msg;
    sensor_msgs::Image rgb_msg;

    /*
      DEFINE SENSOR MESSAGE CONTENTS
    */
    
    //std::cout<<"waiting for frames";
    //auto start = std::chrono::high_resolution_clock::now();
    frames = p.wait_for_frames();
    //auto end = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double, std::milli> elapsed = end - start;
    print_time_stamp("gt_frm");
    //std::cout << "Waited " << elapsed.count() << "ms\t";
    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::video_frame color = frames.get_color_frame();

    unsigned int width = depth.get_width();
    unsigned int height = depth.get_height();

    unsigned int width_color = color.get_width();
    unsigned int height_color = color.get_height();

    //std::cout<<"w/h/w_c/h_c="<<width<<" "<<height<<" "<<width_color<<" "<<height_color<<"\t";
    
    const uint8_t* pixel_ptr = (const uint8_t*)(depth.get_data());
    const uint8_t* pixel_ptr_color = (const uint8_t*)(color.get_data());

    
    unsigned int pixel_amount = width*height;
    unsigned int pixel_amount_color = width_color*height_color;
	
    std::vector<uint8_t> depth_image(2*pixel_amount);
    std::vector<uint8_t> color_image(3*pixel_amount_color);
    memcpy(&depth_image[0], pixel_ptr, 2*pixel_amount*sizeof(uint8_t));
    memcpy(&color_image[0], pixel_ptr_color, 3*pixel_amount_color*sizeof(uint8_t));
    print_time_stamp("cpy_ok");
    
    depth_msg.header.seq = seq;
    depth_msg.header.stamp = ros::Time::now();
    depth_msg.data = depth_image;
    depth_msg.height = height;
    depth_msg.width = width;
    depth_msg.encoding = sensor_msgs::image_encodings::MONO16;
    depth_msg.step = width*2; //each pixel is 2 bytes, so step, which stands for row length in bytes, should be two times "width".

    rgb_msg.header.seq = seq;
    rgb_msg.header.stamp = ros::Time::now();
    rgb_msg.data = color_image;
    rgb_msg.height = height_color;
    rgb_msg.width = width_color;
    rgb_msg.encoding = sensor_msgs::image_encodings::RGB8;
    rgb_msg.step = width_color*3;//each pixel is 3 bytes - red, green, and blue

    print_time_stamp("msg_ok");
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
    rgb_pub.publish(rgb_msg);

    print_time_stamp("pblsh_ok");

    ros::spinOnce();
    seq++;

    loop_rate.sleep();

    //final line
    std::cout<<std::endl;
  }
}


//this function prints a short comment plus the timestamp in [us] since epoch to the screen
//for sake of a clean terminal, please make the comment as shrt as pssbl!
inline void print_time_stamp(std::string comment){
#ifdef PRNT_T_STMP
  std::cout<< comment<<" "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()<<"\t";
#endif //PRNT_T_STMP
  return;
}
