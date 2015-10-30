/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <pcl_texture_mapping/texture_mapper.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "texture_mapper");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  TextureMapper tm(nh, nhp);
  ros::shutdown();
}
