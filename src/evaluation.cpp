/* Calculate omission and commision errors  by comparing the already Ground
   Classified Point Cloud and
    the Ground Filter Point Cloud.

    Dependencies: PCL 1.6
*/
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <vector>
#include <fstream>

#define round(x) ((x < 0) ? (ceil((x)-0.5)) : (floor((x) + 0.5)))

int main(int argc, char const* argv[]) {
  // Read input
  std::string input_pcd = argv[1];
  std::string classified_pcd = argv[2];
  std::string filter_pcd = argv[3];

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground_classified(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground_filter(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  // Load Point Cloud
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZRGB>(input_pcd, *cloud);
  reader.read<pcl::PointXYZRGB>(classified_pcd, *cloud_ground_classified);
  reader.read<pcl::PointXYZRGB>(filter_pcd, *cloud_ground_filter);

  std::vector<int> commission_indices;
  std::vector<int> omission_indices;
  std::vector<int> matching_indices;

  bool ok;

  // Detect omission error
  // How many ground points have been mistakenly removed

  for (size_t i = 0; i < cloud_ground_classified->points.size(); ++i) {
    ok = false;
    for (size_t j = 0; j < cloud_ground_filter->points.size(); ++j) {
      if ((cloud_ground_classified->points[i].x ==
           cloud_ground_filter->points[j].x) &&
          (cloud_ground_classified->points[i].y ==
           cloud_ground_filter->points[j].y) &&
          (cloud_ground_classified->points[i].z ==
           cloud_ground_filter->points[j].z)) {
        matching_indices.push_back(j);

        ok = true;

        break;
      }
    }
    if (!ok) {
      omission_indices.push_back(i);
    }
  }
  // Output omission points
  std::cout << std::endl;
  std::cout << "Omision error points: " << std::endl;
  std::cout << "[How many ground points have been mistakenly removed?]"
            << std::endl;
  std::cout << omission_indices.size() << std::endl;
  std::cout << "_______________________" << std::endl;

  // Detect commission error
  // How many non-ground points have been classified as ground

  for (size_t i = 0; i < cloud_ground_filter->points.size(); ++i) {
    ok = false;
    for (size_t j = 0; j < matching_indices.size(); ++j) {
      if (i == matching_indices[j]) {
        ok = true;
        break;
      }
    }
    if (!ok) {
      commission_indices.push_back(i);
    }
  }

  // Output commission points
  std::cout << std::endl;
  std::cout << "Commision error points: " << std::endl;
  std::cout << "[How many non-ground points have been classified as ground?]"
            << std::endl;
  std::cout << commission_indices.size() << std::endl;
  std::cout << "_______________________" << std::endl;

  char answer;

  int omission_error_points = omission_indices.size();
  int commission_error_points = commission_indices.size();
  int classified_ground_points = cloud_ground_classified->points.size();
  int total_points = cloud->points.size();
  float type_1_error =
      (float)omission_error_points / (float)classified_ground_points * 100;
  float type_2_error = (float)commission_error_points /
                       (float)(total_points - classified_ground_points) * 100;
  float total_error = (float)(omission_error_points + commission_error_points) /
                      (float)total_points * 100;

  std::cout << "------------------" << std::endl;
  std::cout << "Type I error: " << round(type_1_error * 10) / 10 << "%"
            << std::endl;
  std::cout << "Type II error: " << round(type_2_error * 10) / 10 << "%"
            << std::endl;
  std::cout << "Total error: " << round(total_error * 10) / 10 << "%"
            << std::endl;

  do {
    std::cout << "Do you want the errors in a txt "
                 "file?(Y/N)\n";
    std::cin >> answer;
  } while ((answer != 'y') && (answer != 'Y') && (answer != 'n') &&
           (answer != 'N'));

  if ((answer == 'Y') || (answer == 'y')) {
    std::string errors_filename = filter_pcd;
    errors_filename =
        errors_filename.replace(filter_pcd.length() - 4, 11, "_errors.txt");
    std::ofstream errors_file;
    errors_file.open(errors_filename);
    errors_file << "Type I error: " << round(type_1_error * 10) / 10
                << "%" << std::endl;
    errors_file << "Type II error:  " << round(type_2_error * 10) / 10
                << "%" << std::endl;
    errors_file << "Total error:  " << round(total_error * 10) / 10 << "%"
                << std::endl;

    errors_file.close();

    std::cout << "Txt file has been generated.\n";
  }

  return 0;
}