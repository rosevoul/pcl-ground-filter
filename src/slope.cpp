#include "slope.h"

/* Slope estimation using ESRI's 3D Analyst Method */
float slope(pcl::PointCloud<pcl::PointXYZRGB>& cloud, float cell_size) {
  // Set up minimum surface grid
  Eigen::Vector4f global_max, global_min;
  pcl::getMinMax3D<pcl::PointXYZRGB>(cloud, global_min, global_max);

  float xextent = global_max.x() - global_min.x();
  float yextent = global_max.y() - global_min.y();

  int rows = static_cast<int>(std::floor(yextent / cell_size) + 1);
  int cols = static_cast<int>(std::floor(xextent / cell_size) + 1);

  Eigen::MatrixXf A(rows, cols);
  A.setConstant(std::numeric_limits<float>::quiet_NaN());

  for (int i = 0; i < (int)cloud.points.size(); ++i) {
    // ...then test for lower points within the cell
    pcl::PointXYZRGB p = cloud.points[i];
    int row = std::floor((p.y - global_min.y()) / cell_size);
    int col = std::floor((p.x - global_min.x()) / cell_size);

    if (p.z < A(row, col) || pcl_isnan(A(row, col))) {
      A(row, col) = p.z;
    }
  }

  // Estimate slope
  Eigen::MatrixXf Z(rows, cols);
  Z.setConstant(std::numeric_limits<float>::quiet_NaN());

  for (int row = 0; row < rows; ++row) {
    int row_before = row - 1;
    int row_after = row + 1;
    if (row_before < 0)
      row_before = 0;
    if (row_after > rows - 1)
      row_after = rows - 1;
    for (int col = 0; col < cols; ++col) {
      int col_before = col - 1;
      int col_after = col + 1;
      if (col_before < 0)
        col_before = 0;
      if (col_after > cols - 1)
        col_after = cols - 1;

      float a = A(row_before, col_before);
      float b = A(row_before, col);
      float c = A(row_before, col_after);
      float d = A(row, col_before);
      float e = A(row, col);
      float f = A(row, col_after);
      float g = A(row_after, col_before);
      float h = A(row_after, col);
      float i = A(row_after, col_after);

      float dz_dx = ((c + 2 * f + i) - (a + 2 * d + g)) / 8 * cell_size;
      float dz_dy = ((g + 2 * h + i) - (a + 2 * b + c)) / 8 * cell_size;

      double rise_run = sqrt(dz_dx * dz_dx + dz_dy * dz_dy);
      double slope_degrees = atan(rise_run) * 57.29578;

      if (pcl_isnan(Z(row, col)))
        Z(row, col) = slope_degrees;
    }
  }

  float sum = 0.0f;
  int counter = 0;
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      if (Z(row, col) != std::numeric_limits<float>::quiet_NaN()) {
        if (Z(row, col) == Z(row, col)) {
          sum += Z(row, col);
          counter = counter + 1;
        }
      }
    }
  }

  return round(tan(sum / counter * PI / 180.0) * 10) / 10;
}
