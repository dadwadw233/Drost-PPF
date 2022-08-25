//
// Created by yyh on 22-8-18.
//

#ifndef DROST_PPF_PPFREGISTRATION_H
#define DROST_PPF_PPFREGISTRATION_H
#include "Hashmap.hpp"
#include "omp.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
namespace PPF {

class PPFRegistration {
 public:
  PPFRegistration(const pcl::PointCloud<pcl::PointNormal>::Ptr &scene,
                  const pcl::PointCloud<pcl::PointNormal>::Ptr &model,
                  const Hash::HashMap_<Hash::HashKey, Hash::HashData,
                                       Hash::hash_cal>::Ptr &search_map,
                  const Hash::HashMap_<Hash::Trans_key, Hash::Trans_data,
                                       Hash::Tran_cal>::Ptr &Trans_map)
      : scene_cloud_with_normal(scene),
        model_cloud_with_normal(model),
        searchMap(search_map),
        model_trans(Trans_map){};

  PPFRegistration();

  void compute();

  Eigen::Affine3f getFinalTransformation();

  void setSceneReferencePointSamplingRate(
      const float &scene_reference_point_sampling_rate);

  void setPositionClusteringThreshold(
      const float &clustering_position_diff_threshold);

  void setRotationClusteringThreshold(
      const float &clustering_rotation_diff_threshold);

  void setDiscretizationSteps(const float &angle_discretization_step,
                              const float &distance_discretization_step);

  void setSearchMap(const Hash::HashMap_<Hash::HashKey, Hash::HashData,
                                         Hash::hash_cal>::Ptr &searchMap);

  void setTransMap(const Hash::HashMap_<Hash::Trans_key, Hash::Trans_data,
                                        Hash::Tran_cal>::Ptr &Trans);

  void setInputSource(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

  void setInputTarget(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

  void setDobj(const float &data);

  PPFRegistration &operator=(const PPFRegistration &) = delete;
  PPFRegistration(const PPFRegistration &) = delete;

 private:
  struct Accumulator{
    int value;
    Eigen::Affine3f T{};
    Accumulator(){
      value = 0;
    }
    Accumulator(const int &value_, const Eigen::Affine3f &T_):value(value_),T(T_){};

  };
  bool check();
  float scene_reference_point_sampling_rate = 0;
  float clustering_position_diff_threshold = 0;
  float clustering_rotation_diff_threshold = 0;

  pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud_with_normal;
  pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud_with_normal;
  Eigen::Affine3f finalTransformation;
  Hash::HashMap_<Hash::HashKey, Hash::HashData, Hash::hash_cal>::Ptr searchMap;
  Hash::HashMap_<Hash::Trans_key, Hash::Trans_data, Hash::Tran_cal>::Ptr
      model_trans;
  std::vector<std::vector<Accumulator>> accumulatorSpace;

  float angle_discretization_step = 0;
  float distance_discretization_step = 0;
  float d_obj = 0;
};

}  // namespace PPF

#endif  // DROST_PPF_PPFREGISTRATION_H
