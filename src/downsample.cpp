#include "downsample.h"

void downsample(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud,
                pcl::PointCloud<pcl::PointXYZRGB>& downsampled_cloud,
                float leaf_size) {
 
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
	
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid.setInputCloud(input_cloud);
  voxel_grid.filter(downsampled_cloud); 
}