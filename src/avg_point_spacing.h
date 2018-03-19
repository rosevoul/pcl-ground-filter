#ifndef AVG_POINT_SPACING_H
#define AVG_POINT_SPACING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <numeric>

float calc_avg_point_spacing(pcl::PointCloud<pcl::PointXY>& inputCloud);

#endif
