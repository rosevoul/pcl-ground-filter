#ifndef DOWNSAMPLE_H
#define DOWNSAMPLE_H

#include "pcl/octree/octree.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

void downsample(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud,
                pcl::PointCloud<pcl::PointXYZRGB>& downsampled_cloud,
                float leaf_size);

#endif