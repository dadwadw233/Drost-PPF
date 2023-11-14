//
// Created by yyh on 22-8-18.
//

#include "PPFEstimation.h"
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "PPFEstimation.h"
#include "chrono"
#include"cmath"
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
    Eigen::Vector3f x_n{1, 0, 0};
    auto alpha =
        pcl::getAngle3D(Eigen::Vector3f(input_point_normal->points[i].normal_x,
                                        input_point_normal->points[i].normal_y,
                                        input_point_normal->points[i].normal_z),
                        x_n);
    Eigen::Vector3f t{
        -input_point_normal->points[i].x, -input_point_normal->points[i].y,
        -input_point_normal->points[i].z};  // transition between mr and O
    Eigen::Vector3f n_ =
        (Eigen::Vector3f(input_point_normal->points[i].normal_x,
                         input_point_normal->points[i].normal_y,
                         input_point_normal->points[i].normal_z)
             .cross(x_n))
            .normalized();
    Eigen::AngleAxisf v(static_cast<float>(alpha), n_);

    Eigen::Matrix3f R;
    R << v.matrix();
    Eigen::Matrix4f T;
    T << R(0, 0), R(0, 1), R(0, 2), t[0], R(1, 0), R(1, 1), R(1, 2), t[1],
        R(2, 0), R(2, 1), R(2, 2), t[2], 0, 0, 0, 1;
    Eigen::Affine3f T_(T);
    model_trans->addInfo(PPF::Hash::Trans_key(input_point_normal->points[i]),
                         PPF::Hash::Trans_data(T_));
#pragma omp parallel shared(R, input_point_normal, hash_map, \
                            i) private(data, p1, p2, n1, n2,                \
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
          delta.normalize();
          Eigen::Vector3f d = delta.normalized();
          d = R * d;
          Eigen::Vector3f x{1, 0, 0};
          Eigen::Vector3f z{0, 0, 1};
          Eigen::Vector3f y{0, 1, 0};

          double model_alpha =
              acos(fabs(d.cross(x).dot(y)) / (d.cross(x).norm() * y.norm()));//求取二面角

          if (fabs(pcl::getAngle3D(d, z, true)) <= 90 &&
              fabs(pcl::getAngle3D(d, y, true)) <= 90) {
            model_alpha = model_alpha;
          } else if (fabs(pcl::getAngle3D(d, z, true)) <= 90 &&
                     fabs(pcl::getAngle3D(d, y, true)) >= 90) {
            model_alpha = 2 * M_PI - model_alpha;
          } else if (fabs(pcl::getAngle3D(d, z, true)) >= 90 &&
                     fabs(pcl::getAngle3D(d, y, true)) >= 90) {
            model_alpha = M_PI + model_alpha;
          } else {
            model_alpha = M_PI - model_alpha;
          }//按照所处不同的象限进行分类
          data.second.angle = model_alpha;



          float f1 = atan2(delta.cross(n1).norm(), delta.dot(n1));

          float f2 = atan2(delta.cross(n2).norm(), delta.dot(n2));

          float f3 = atan2(n1.cross(n2).norm(), n1.dot(n2));

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
          data.second.index = i;
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