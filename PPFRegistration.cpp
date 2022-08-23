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

PPFRegistration::PPFRegistration() {
  this->model_cloud_with_normal =
      boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  this->scene_cloud_with_normal =
      boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  this->searchMap = boost::make_shared<
      Hash::HashMap_<Hash::HashKey, Hash::HashData, Hash::hash_cal>>();
  this->model_trans = boost::make_shared<
      Hash::HashMap_<Hash::Trans_key, Hash::Trans_data, Hash::Tran_cal>>();
}
bool PPFRegistration::check() {
  if (this->model_cloud_with_normal->points.empty() ||
      this->scene_cloud_with_normal->points.empty() ||
      this->searchMap->empty() || this->model_trans->empty()) {
    PCL_ERROR("Neither point cloud or search map/trans map should be empty!\n");
    PCL_ERROR("Init failed, make sure all params are initialized and retry\n");
    return false;
  } else if (!scene_reference_point_sampling_rate ||
             !clustering_position_diff_threshold ||
             !clustering_rotation_diff_threshold) {
    PCL_ERROR("Params for clustering are not be initialized\n");
    PCL_ERROR("Init failed, make sure all params are initialized and retry\n");
    return false;
  } else if (!angle_discretization_step || !distance_discretization_step) {
    PCL_ERROR("Params for PPF establishment are not be initialized\n");
    PCL_ERROR("Init failed, make sure all params are initialized and retry\n");
    return false;
  } else {
    PCL_INFO("Pass init check");
    return true;
  }
}
}  // namespace PPF