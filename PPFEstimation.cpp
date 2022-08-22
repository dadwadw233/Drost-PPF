//
// Created by yyh on 22-8-18.
//

#include "PPFEstimation.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "PPFEstimation.h"
#include "chrono"
#include "math.h"
#include "omp.h"
namespace PPF {

void PPFEstimation::compute(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &input_point_normal,
    Hash::HashMap_<Hash::HashKey, Hash::HashData, Hash::hash_cal>::Ptr
        &hash_map,
    Hash::HashMap_<Hash::Trans_key, Hash::Trans_data, Hash::Tran_cal>::Ptr
        &model_trans) {
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
                            i) private(data, p1, p2, n1, n2,                  \
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

          Eigen::Vector3f x_n{1, 0, 0};
          std::cout << "before:" << std::endl
                    << pcl::getAngle3D(n1, x_n, true) << std::endl;
          auto alpha = pcl::getAngle3D(n1, x_n);
          Eigen::Vector3f t{
              -input_point_normal->points[i].x,
              -input_point_normal->points[i].y,
              -input_point_normal->points[i].z};  // transition between mr and O
          // Eigen::Vector3f n_ = (x_n.cross(n1)).normalized();
          Eigen::Vector3f n_ = (n1.cross(x_n)).normalized();
          Eigen::AngleAxisf v(static_cast<float>(alpha), n_);

          Eigen::Matrix3f R;
          R << v.matrix();
          /* auto M = this->rot_mat(Eigen::Vector3f
           {input_point_normal->points[i].x, input_point_normal->points[i].y,
           input_point_normal->points[i].z}, n_, model_alpha); R<<M(0,0),
           M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2);
           */
          Eigen::Vector3f after{};
          after = R * n1;
          // std::cout<<"after:"<<std::endl<<pcl::getAngle3D(after, x_n,
          // true)<<std::endl;
          Eigen::Matrix4f T;
          T << R(0, 0), R(0, 1), R(0, 2), t[0], R(1, 0), R(1, 1), R(1, 2), t[1],
              R(2, 0), R(2, 1), R(2, 2), t[2], 0, 0, 0, 1;
          Eigen::Affine3f T_(T);

          Eigen::Vector3f after_;
          after_ = R * n1;
          std::cout << "after:" << std::endl
                    << pcl::getAngle3D(after, x_n, true) << std::endl;

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

}  // namespace PPF