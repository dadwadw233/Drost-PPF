//
// Created by yyh on 22-8-18.
//

#ifndef DROST_PPF_PPFESTIMATION_H
#define DROST_PPF_PPFESTIMATION_H
#include "Eigen/Core"
#include "Hashmap.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
namespace PPF {

class PPFEstimation {
  PPFEstimation();
  void compute(const pcl::PointCloud<pcl::PointNormal>::Ptr &input_point_normal,
               pcl::PointCloud<pcl::PPFSignature>::Ptr &output_cloud,
               Hash::HashMap::Ptr &hash_map);

  void setDiscretizationSteps(const float &angle_discretization_step,
                              const float &distance_discretization_step);

  PPFEstimation &operator=(const PPFEstimation &) = delete;
  PPFEstimation(const PPFEstimation &) = delete;

 private:
  float angle_discretization_step;
  float distance_discretization_step;

};

}  // namespace PPF

#endif  // DROST_PPF_PPFESTIMATION_H
