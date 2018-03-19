#include "max_elevation_diff.h"

float max_elevation_diff(const pcl::PointCloud<pcl::PointXYZRGB> & 	inputCloud){

pcl::PointXYZRGB min_point, max_point;
getMinMax3D(inputCloud, min_point, max_point);
return max_point.z - min_point.z;
}


