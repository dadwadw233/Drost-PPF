//
// Created by yyh on 22-8-18.
//

#include "Drost_PPF.h"

template <class T>
decltype(auto) Drost_PPF::transform(typename pcl::PointCloud<T>::Ptr &input_cloud){
  return pcl::transformPoint(input_cloud,transformMat);
}
PPF::Drost_PPF::Drost_PPF(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_model,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_scene) {}
