#ifndef UPSAMPLE_H
#define UPSAMPLE_H

#include "pcl/octree/octree.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

void upsample(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud,
				const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& downsampled_cloud,
                pcl::PointCloud<pcl::PointXYZRGB>& upsampled_cloud,
                float resolution);

#endif