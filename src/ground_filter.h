#ifndef GROUND_FILTER_H
#define GROUND_FILTER_H

#include <iostream>
#include <fstream>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

void ground_filter(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inputCloud,
    pcl::PointCloud<pcl::PointXYZRGB>& groundCloud,
	float max_distance,
	float cell_size, 
	float slope);

#endif