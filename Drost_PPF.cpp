//
// Created by yyh on 22-8-18.
//

#include "Drost_PPF.h"
namespace PPF {

decltype(auto) Drost_PPF::transform(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr out;
  pcl::transformPointCloud(*input_cloud, *out, transformMat);
  return out;
}
Drost_PPF::Drost_PPF(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_model,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_scene) {
  this->model_set.push_back(input_model);
  ++model_num;
  this->scene = input_scene;
  sceneSet = true;
}
Drost_PPF::Drost_PPF(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_model) {
  this->model_set.push_back(input_model);
  ++model_num;
}
void Drost_PPF::setSceneCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_scene) {
  this->scene = input_scene;
  sceneSet = true;
}
void Drost_PPF::addModelCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_model) {
  this->model_set.push_back(input_model);
  ++model_num;
}
decltype(auto) Drost_PPF::getTransform() { return transformMat; }
decltype(auto) Drost_PPF::subsampleAndCalculateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud(cloud);
  subsampling_filter.setLeafSize(leaf_size);
  subsampling_filter.filter(*cloud_subsampled);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_subsampled_normals(
      new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud(cloud_subsampled);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimation_filter.setSearchMethod(search_tree);
  normal_estimation_filter.setKSearch(k_point);
  // normal_estimation_filter.setRadiusSearch(normalEstimationRadius);
  normal_estimation_filter.compute(*cloud_subsampled_normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_subsampled_with_normals(
      new pcl::PointCloud<pcl::PointNormal>());
  concatenateFields(*cloud_subsampled, *cloud_subsampled_normals,
                    *cloud_subsampled_with_normals);

  PCL_INFO("Cloud dimensions before / after subsampling: %zu / %zu\n",
           static_cast<std::size_t>(cloud->size()),
           static_cast<std::size_t>(cloud_subsampled->size()));
  return cloud_subsampled_with_normals;
}
bool Drost_PPF::check() {
  if (model_num == 0) {
    PCL_ERROR("You need to add model point cloud first!");
    return false;
  } else if (!sceneSet) {
    PCL_ERROR("You need to add scene point cloud first!");
    return false;
  } else if (k_point == 0) {
    PCL_ERROR("Parameters for normal estimation are not set");
    return false;
  } else if (leaf_size[0] == 0 || leaf_size[1] == 0 || leaf_size[2] == 0) {
    PCL_ERROR("Parameters for downsampling are not set");
    return false;
  } else if (angle_discretization_step == 0 ||
             distance_discretization_step == 0) {
    PCL_ERROR("Parameters for PPF discrete are not set");
    return false;
  } else {
    return true;
  }
}
void Drost_PPF::solve() {
  if (!check()) {
    PCL_ERROR(
        "The program has been interrupted due to unset or incorrectly set "
        "parameters！Please check and retry！");
    return;
  } else {
  }
}
void Drost_PPF::setLeafSize(const Eigen::Vector4f &leaf) {
  if (leaf[0] == 0 || leaf[1] == 0 || leaf[2] == 0) {
    PCL_ERROR(
        "The first three dimensions of leaf_size should not be set to 0！");
    return;
  } else {
    this->leaf_size = leaf;
    return;
  }
}
void Drost_PPF::setKPoint(const int &num) {
  if (!num) {
    PCL_ERROR("The k_point should not be set to 0！");
    return;
  } else {
    this->k_point = num;
    return;
  }
}
void Drost_PPF::setPPFDiscretizationStep(const float &angle,
                                         const float &distance) {
  this->angle_discretization_step = angle;
  this->distance_discretization_step = distance;
  return;
}
void Drost_PPF::test() {
  PPFEstimation estimator{};
  this->setLeafSize(Eigen::Vector4f(13, 13, 13, 0));
  this->setKPoint(10);
  auto model_with_normals = subsampleAndCalculateNormals(model_set[0]);
  PPF::Hash::HashMap_<Hash::HashKey, Hash::HashData, Hash::hash_cal>::Ptr
      hashmap = boost::make_shared<
          PPF::Hash::HashMap_<Hash::HashKey, Hash::HashData, Hash::hash_cal>>();
  PPF::Hash::HashMap_<Hash::Trans_key, Hash::Trans_data,
                      Hash::Tran_cal>::Ptr model_trans =
      boost::make_shared<PPF::Hash::HashMap_<Hash::Trans_key, Hash::Trans_data,
                                             Hash::Tran_cal>>();
  estimator.setDiscretizationSteps(12.0f / 180.0f * float(M_PI), 0.05f);
  estimator.compute(model_with_normals, hashmap, model_trans);
}
}  // namespace PPF
