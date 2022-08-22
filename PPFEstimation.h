//
// Created by yyh on 22-8-18.
//

#ifndef DROST_PPF_PPFESTIMATION_H
#define DROST_PPF_PPFESTIMATION_H
#include "Eigen/Core"
#include "Hashmap.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace PPF {

class PPFEstimation {
 public:
  PPFEstimation() = default;
  void compute(const pcl::PointCloud<pcl::PointNormal>::Ptr &input_point_normal,
               Hash::HashMap_<Hash::HashKey, Hash::HashData,
                              Hash::hash_cal>::Ptr &hash_map,
               Hash::HashMap_<Hash::Trans_key, Hash::Trans_data,
                              Hash::Tran_cal>::Ptr &model_trans);

  void setDiscretizationSteps(const float &angle_discretization_step,
                              const float &distance_discretization_step);

  PPFEstimation &operator=(const PPFEstimation &) = delete;
  PPFEstimation(const PPFEstimation &) = delete;

 private:
  // Returns the rotation matrix around a vector  placed at a point , rotate by
  // angle t
  Eigen::Matrix4f rot_mat(const Eigen::Vector3f &point,
                          const Eigen::Vector3f &vector, const float t) {
    float u = vector(0);
    float v = vector(1);
    float w = vector(2);
    float a = point(0);
    float b = point(1);
    float c = point(2);

    Eigen::Matrix4f matrix;
    matrix << u * u + (v * v + w * w) * cos(t),
        u * v * (1 - cos(t)) - w * sin(t), u * w * (1 - cos(t)) + v * sin(t),
        (a * (v * v + w * w) - u * (b * v + c * w)) * (1 - cos(t)) +
            (b * w - c * v) * sin(t),
        u * v * (1 - cos(t)) + w * sin(t), v * v + (u * u + w * w) * cos(t),
        v * w * (1 - cos(t)) - u * sin(t),
        (b * (u * u + w * w) - v * (a * u + c * w)) * (1 - cos(t)) +
            (c * u - a * w) * sin(t),
        u * w * (1 - cos(t)) - v * sin(t), v * w * (1 - cos(t)) + u * sin(t),
        w * w + (u * u + v * v) * cos(t),
        (c * (u * u + v * v) - w * (a * u + b * v)) * (1 - cos(t)) +
            (a * v - b * u) * sin(t),
        0, 0, 0, 1;
    return matrix;
  }
  float angle_discretization_step;
  float distance_discretization_step;
};

}  // namespace PPF

#endif  // DROST_PPF_PPFESTIMATION_H
