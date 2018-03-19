#include "ground_filter.h"

void ground_filter(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inputCloud,
    pcl::PointCloud<pcl::PointXYZRGB>& groundCloud,
	float max_distance,
	float cell_size, 
	float slope) {
  pcl::PointIndicesPtr ground(new pcl::PointIndices);

  // Progressive Morphological Filter
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZRGB> pmf;
  pmf.setInputCloud(inputCloud);

  pmf.setMaxWindowSize(33);
  pmf.setMaxDistance(max_distance);
  pmf.setInitialDistance(0.15f);
  pmf.setCellSize(cell_size);
  pmf.setSlope(slope);
  pmf.setBase(2.0f);
  pmf.setExponential(true);

  pmf.extract(ground->indices);

  // Create the filtering object and extract the ground returns
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(inputCloud);
  extract.setIndices(ground);
  extract.filter(groundCloud);
}