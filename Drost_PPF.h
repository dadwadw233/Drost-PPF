//
// Created by yyh on 22-8-18.
//

#ifndef DROST_PPF_DROST_PPF_H
#define DROST_PPF_DROST_PPF_H
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include "Eigen/Core"
#include "Hashmap.hpp"
#include "PPFEstimation.h"
#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_representation.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
namespace PPF {
class Drost_PPF {
 public:
  Drost_PPF(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_model,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_scene);
  Drost_PPF(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_model);

  void setSceneCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_scene);

  void setPPFDiscretizationStep(const float &angle, const float &distance);

  void addModelCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_model);

  void setLeafSize(const Eigen::Vector4f &leaf);

  void setKPoint(const int &num);

  void solve();

  decltype(auto) transform(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);

  decltype(auto) getTransform();

  void test();

 private:
  decltype(auto) subsampleAndCalculateNormals(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

  bool check();

  int model_num = 0;
  int k_point = 0;

  float angle_discretization_step = 0;
  float distance_discretization_step = 0;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> model_set;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene;

  bool sceneSet = false;
  Eigen::Affine3f transformMat;
  Eigen::Vector4f leaf_size{0.0f, 0.0f, 0.0f, 0.0f};
};

}  // namespace PPF

#endif  // DROST_PPF_DROST_PPF_H
