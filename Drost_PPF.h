//
// Created by yyh on 22-8-18.
//

#ifndef DROST_PPF_DROST_PPF_H
#define DROST_PPF_DROST_PPF_H
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/point_representation.h"
#include "pcl/visualization/cloud_viewer.h"
#include "Eigen/Core"
namespace PPF{
class Drost_PPF {
 public:
  Drost_PPF(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_model, const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_scene);
  Drost_PPF(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_model);

  void setSceneCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_scene);

  void addModelCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_model);

  void solve();

  template <class T>
  decltype(auto) transform(typename pcl::PointCloud<T>::Ptr &input_cloud);

  decltype(auto) getTransform();


 private:
  decltype(auto) subsampleAndCalculateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);


  int model_num = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> model_set;

  Eigen::Affine3f transformMat;
};

}

#endif  // DROST_PPF_DROST_PPF_H
