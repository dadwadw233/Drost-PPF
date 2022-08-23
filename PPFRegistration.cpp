//
// Created by yyh on 22-8-18.
//

#include "PPFRegistration.h"

namespace PPF {
Eigen::Affine3f PPFRegistration::getFinalTransformation() {
  return this->finalTransformation;
}
void PPFRegistration::setSceneReferencePointSamplingRate(
    const float& scene_reference_point_sampling_rate) {
  this->scene_reference_point_sampling_rate =
      scene_reference_point_sampling_rate;
}
void PPFRegistration::setPositionClusteringThreshold(
    const float& clustering_position_diff_threshold) {
  this->clustering_position_diff_threshold = clustering_position_diff_threshold;
}
void PPFRegistration::setRotationClusteringThreshold(
    const float& clustering_rotation_diff_threshold) {
  this->clustering_rotation_diff_threshold = clustering_rotation_diff_threshold;
}
void PPFRegistration::setDiscretizationSteps(
    const float& angle_discretization_step,
    const float& distance_discretization_step) {
  this->angle_discretization_step = angle_discretization_step;
  this->distance_discretization_step = distance_discretization_step;
}

void PPFRegistration::setDobj(const float& data) { this->d_obj = data; }

void PPFRegistration::setSearchMap(
    const Hash::HashMap_<Hash::HashKey, Hash::HashData, Hash::hash_cal>::Ptr&
        searchMap) {
  this->searchMap = searchMap;
}
void PPFRegistration::setTransMap(
    const Hash::HashMap_<Hash::Trans_key, Hash::Trans_data,
                         Hash::Tran_cal>::Ptr& Trans) {
  this->model_trans = Trans;
}
void PPFRegistration::setInputSource(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) {
  this->model_cloud_with_normal = cloud;
}
void PPFRegistration::setInputTarget(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) {
  this->scene_cloud_with_normal = cloud;
}
void PPFRegistration::compute() {}
}  // namespace PPF