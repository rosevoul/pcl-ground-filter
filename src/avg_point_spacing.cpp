/*Average Point Spacing calculation using the K-nearest neighbor search
  ISSUES: Random selection of points in case of big data, instead of the first
  25000(point limit) points.
  */
#include "avg_point_spacing.h"

float calc_avg_point_spacing(pcl::PointCloud<pcl::PointXY>& inputCloud) {
  // Check number of points
  int point_limit = 25000;
  int iteration_limit;
  if (inputCloud.points.size() > point_limit)
    iteration_limit = point_limit;
  else
    iteration_limit = inputCloud.points.size();

  // K nearest neighbor search
  // -- K = 1 gives the same point
  // -- K = 2 gives the closest neighbor
  pcl::PointCloud<pcl::PointXY>::Ptr input_cloud_ptr(&inputCloud);
  pcl::KdTreeFLANN<pcl::PointXY> kdtree;
  kdtree.setInputCloud(input_cloud_ptr);
  int K = 2;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  std::vector<float> neighborSquaredDistance;

  for (size_t j = 0; j < iteration_limit; ++j) {
    if (kdtree.nearestKSearch(inputCloud.points[j], K, pointIdxNKNSearch,
                              pointNKNSquaredDistance) > 0)
      neighborSquaredDistance.push_back(pointNKNSquaredDistance[1]);
  }
  float squared_avg = std::accumulate(neighborSquaredDistance.begin(),
                                      neighborSquaredDistance.end(), 0.0) /
                      neighborSquaredDistance.size();

  return sqrt(squared_avg);
}