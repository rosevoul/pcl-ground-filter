#include "upsample.h"

void upsample(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& downsampled_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>& upsampled_cloud,
    float resolution) {
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution);
  octree.setInputCloud(input_cloud);
  octree.addPointsFromInputCloud();

  pcl::PointXYZRGB searchPoint;

  // Fill in the input point cloud data

  for (size_t i = 0; i < downsampled_cloud->points.size(); ++i) {
    searchPoint = downsampled_cloud->points[i];
    //upsampled_cloud.points.push_back(searchPoint);
    std::vector<int> pointIdxVec;
    if (octree.voxelSearch(searchPoint, pointIdxVec)) {
      for (size_t i = 0; i < pointIdxVec.size(); ++i) {
        //float elevation_diff =
           // std::abs(input_cloud->points[pointIdxVec[i]].z - searchPoint.z);
        //if (elevation_diff < 0.05)
          upsampled_cloud.points.push_back(input_cloud->points[pointIdxVec[i]]);
      }
    }
  }

  upsampled_cloud.width = upsampled_cloud.points.size();
  upsampled_cloud.height = 1;
}