//
// Created by yyh on 22-8-18.
//

#include "PPFEstimation.h"
#include "PPFEstimation.h"
#include "chrono"
#include "omp.h"
namespace PPF {

void PPFEstimation::compute(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &input_point_normal,
    pcl::PointCloud<pcl::PPFSignature>::Ptr &output_cloud,
    Hash::Ptr &hash_map) {

  pcl::PPFSignature feature{};
  std::pair<Hash::HashKey, Hash::HashData> data{};
  Eigen::Vector3f p1{};
  Eigen::Vector3f p2{};
  Eigen::Vector3f n1{};
  Eigen::Vector3f n2{};
  Eigen::Vector3f delta{};

  auto tp1 = std::chrono::steady_clock::now();

  for (auto i = 0; i < input_point_normal->size(); ++i) {
#pragma omp parallel shared(input_point_normal, output_cloud, hash_map, cout, \
                            i) private(data, p1, p2, n1, n2,         \
                                       delta) default(none)
    {
#pragma omp for
      for (auto j = 0; j < input_point_normal->size(); ++j) {
        if (i == j) {
          continue;
        } else {
          p1 << input_point_normal->points[i].x,
              input_point_normal->points[i].y, input_point_normal->points[i].z;
          p2 << input_point_normal->points[j].x,
              input_point_normal->points[j].y, input_point_normal->points[j].z;
          n1 << input_point_normal->points[i].normal_x,
              input_point_normal->points[i].normal_y,
              input_point_normal->points[i].normal_z;
          n2 << input_point_normal->points[j].normal_x,
              input_point_normal->points[j].normal_y,
              input_point_normal->points[j].normal_z;
          delta = p2 - p1;  // pt-pr
          float f4 = delta.norm();

          delta /= f4;

          float f1 = n1[0] * delta[0] + n1[1] * delta[1] + n1[2] * delta[2];

          float f2 = n1[0] * delta[0] + n2[1] * delta[1] + n2[2] * delta[2];

          float f3 = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];


          data.first.k1 =
              static_cast<int>(std::floor(f1 / angle_discretization_step));
          data.first.k2 =
              static_cast<int>(std::floor(f2 / angle_discretization_step));
          data.first.k3 =
              static_cast<int>(std::floor(f3 / angle_discretization_step));
          data.first.k4 =
              static_cast<int>(std::floor(f4 / distance_discretization_step));

          data.second.r = input_point_normal->points[i];
          data.second.t = input_point_normal->points[j];

#pragma omp critical
          hash_map->addInfo(data);

        }
      }
    }
  }
#pragma omp barrier
  auto tp2 = std::chrono::steady_clock::now();
  std::cout << "need "
            << std::chrono::duration_cast<std::chrono::milliseconds>(tp2 - tp1)
                   .count()
            << "ms to process PPF" << std::endl;
}

void PPFEstimation::setDiscretizationSteps(
    const float &angle_discretization_step,
    const float &distance_discretization_step) {
  this->angle_discretization_step = angle_discretization_step;
  this->distance_discretization_step = distance_discretization_step;
}
PPFEstimation::PPFEstimation() {}

}  // namespace PPF