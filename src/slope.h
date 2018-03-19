#ifndef SLOPE_H
#define SLOPE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>


#define round(x) ((x < 0) ? (ceil((x)-0.5)) : (floor((x) + 0.5)))
#define PI 3.14159265

float slope(pcl::PointCloud<pcl::PointXYZRGB>& cloud, float cell_size) ;

#endif