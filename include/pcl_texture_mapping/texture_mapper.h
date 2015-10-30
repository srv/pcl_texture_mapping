#ifndef TEXTURE_MAPPER_H
#define TEXTURE_MAPPER_H

/// ROS Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>

/// PCL Includes
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/boost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/visualization/pcl_visualizer.h>

/// OpenCV Includes
// #include <opencv2/opencv.hpp>

using namespace pcl;
using namespace std;

typedef PointXYZRGB          PointRGB;
typedef PointCloud<PointRGB> PointCloudRGB;
typedef PointCloud<PointXYZ> PointCloudXYZ;

class TextureMapper
{
 public:
  TextureMapper(ros::NodeHandle nh, ros::NodeHandle nhp);
 private:
  // Node handlers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  double focal_length_;
  int image_height_;
  int image_width_;

  PointCloudRGB::Ptr filterCloud(PointCloudRGB::Ptr in_cloud, float voxel_size);
  void greedyTriangulation(PointCloudRGB::Ptr cloud, PolygonMesh& triangles);
  void transformTFToEigen(const tf::Transform &t, Eigen::Affine3f &e);
  int saveOBJFile (const string &file_name,
                   const TextureMesh &tex_mesh, unsigned precision);
  void mapTexture(PointCloudXYZ::Ptr cloud,
                  const PolygonMesh& triangles,
                  const vector< pair<string, tf::Transform> > &img_poses,
                  TextureMesh& mesh);
  static tf::Transform readPoses(string work_dir, int cloud_id, tf::Transform& cloud_pose);
  void process(string cloud_filename, string work_dir);
  void showCameras (pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
};

#endif  // TEXTURE_MAPPER_H
