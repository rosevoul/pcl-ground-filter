#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "downsample.h"
#include "upsample.h"
#include "ground_filter.h"
#include "avg_point_spacing.h"
#include "slope.h"
#include "max_elevation_diff.h"

#define BIG_DATA_POINT_LIMIT 35000
#define DOWNSAMPLE_FACTOR 45
#define CELL_FACTOR 3

int main(int argc, char const* argv[]) {
  const clock_t begin_time = clock();
  // Welcome screen
  std::cout << "AUTO GROUND FILTER" << std::endl;
  std::cout << "------------------" << std::endl;

  // PCL set-up
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  // Read input
  std::string inputPCD = argv[1];
  reader.read<pcl::PointXYZRGB>(inputPCD, *inputCloud);
  std::cout << "Cloud before filtering: ";
  std::cout << inputCloud->points.size() << " POINTS" << std::endl;
  std::cout << "------------------" << std::endl;

  // Calculate max_distance
  float max_distance_ = 3.0f;
  bool isForest = (!(strcmp("f", argv[2])));
  bool isBuilding = (!(strcmp("b", argv[2])));
  if (isForest) {
    max_distance_ = max_elevation_diff(*inputCloud);
    std::cout << "FOREST AREA" << std::endl;
  }
  else if (isBuilding)
    std::cout << "BUILDING AREA" << std::endl;
  else {
    std::cout << "You did not enter F or B as the second argument!"<< std::endl;
    exit(1);
  }
  std::cout << "max_distance_: " << max_distance_ << std::endl;
  std::cout << "------------------" << std::endl;

  // Calculate average point spacing
  pcl::PointCloud<pcl::PointXY>::Ptr inputCloudXY(
      new pcl::PointCloud<pcl::PointXY>);
  pcl::PCDReader readerXY;
  readerXY.read<pcl::PointXY>(inputPCD, *inputCloudXY);
  float avg_point_spacing = calc_avg_point_spacing(*inputCloudXY);
  
  // Check number of points
  bool isBig = false;
  int point_limit = BIG_DATA_POINT_LIMIT; 
  float resolution = 0;
  if (inputCloud->points.size() > point_limit) {
    isBig = true;    
  }

  // Downsample using octree
  if (isBig) {
    std::cout << "Downsampling..." << std::endl;
	resolution =  DOWNSAMPLE_FACTOR * avg_point_spacing;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    downsample(inputCloud, *downsampledCloud, resolution);

    *filterCloud = *downsampledCloud;   
    std::cout << "Cloud after downsampling: ";
    std::cout << filterCloud->points.size() << " POINTS" << std::endl;
    std::cout << "------------------" << std::endl;
  } else
    *filterCloud = *inputCloud;

  // Calculate average point spacing again!
  pcl::PointCloud<pcl::PointXY>::Ptr downsampledCloudXY(
      new pcl::PointCloud<pcl::PointXY>);

  downsampledCloudXY->points.resize(filterCloud->points.size());
  for (size_t i = 0; i < filterCloud->points.size(); i++) {
    downsampledCloudXY->points[i].x = filterCloud->points[i].x;
    downsampledCloudXY->points[i].y = filterCloud->points[i].y;
  }
  avg_point_spacing = calc_avg_point_spacing(*downsampledCloudXY);
  float cell_size_ = round(avg_point_spacing * CELL_FACTOR * 100) / 100;

  std::cout << "cell_size_ = " << cell_size_ << std::endl;
  std::cout << "------------------" << std::endl;
  
  // Calculate average slope   
  float slope_ = slope(*filterCloud, cell_size_);
  std::cout << "slope_: " << slope_ << std::endl;
  std::cout << "------------------" << std::endl;

  // Ground Filter
  std::cout << "Ground Filtering..." << std::endl;
  ground_filter(filterCloud, *groundCloud, max_distance_, cell_size_, slope_);
  std::cout << "Ground cloud after filtering: ";
  std::cout << groundCloud->points.size() << " POINTS" << std::endl;
  std::cout << "------------------" << std::endl;

  // Upsample using octree
  if (isBig) {
    std::cout << "Upsampling..." << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr upsampledCloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    upsample(inputCloud, groundCloud, *upsampledCloud, resolution);

    std::cout << "Ground cloud after upsampling: ";
    std::cout << upsampledCloud->points.size() << " POINTS" << std::endl;

    *groundCloud = *upsampledCloud;
    std::cout << "------------------" << std::endl;
  }

  // Output
  std::string outputPCD = inputPCD;
  outputPCD.replace(outputPCD.length() - 4, 11, "_ground.pcd");
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB>(outputPCD, *groundCloud, false);

  // Calculate execution time
  std::cout << "Time: " << (float(clock() - begin_time) / CLOCKS_PER_SEC / 60)
            << " minutes." << std::endl;
  return 0;
}