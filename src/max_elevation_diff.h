#ifndef MAX_ELEVATION_DIFF_H
#define MAX_ELEVATION_DIFF_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

float max_elevation_diff(const pcl::PointCloud<pcl::PointXYZRGB > & 	inputCloud);

#endif