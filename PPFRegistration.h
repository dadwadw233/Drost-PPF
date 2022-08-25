//
// Created by yyh on 22-8-18.
//

#ifndef DROST_PPF_PPFREGISTRATION_H
#define DROST_PPF_PPFREGISTRATION_H
#include <pcl/common/common.h>
#include "Hashmap.hpp"
#include "omp.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/common/transforms.h"
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
    int value = 0;
    std::vector<Eigen::Affine3f>T_set;
  };
  bool check();
  void vote(const int &i_, const int &alpha_, const Eigen::Affine3f &T_);
  decltype(auto) getMeanTransform(const int i_, const int alpha_){
    Eigen::Matrix4f sum{};
    //std::cout<<"ok"<<std::endl;
    //std::cout<<accumulatorSpace[i_][alpha_].value<<std::endl;
    for(auto T:accumulatorSpace[i_][alpha_].T_set){
      sum+=T.matrix();
    }
    return Eigen::Affine3f (sum/accumulatorSpace[i_][alpha_].T_set.size());
    //return accumulatorSpace[i_][alpha_].T_set[0];
  }
  decltype(auto) accumulatorSort(){
    int max_value = -1;
    int i = 0;
    int alpha = 0;

    for(auto j = 0;j<accumulatorSpace.size();j++){
      for(auto k = 0;k<accumulatorSpace[j].size();k++){
        //std::cout<<j<<" "<<k<<std::endl;
        if(accumulatorSpace[j][k].value>max_value){
          i = j;
          alpha = k;
          max_value = accumulatorSpace[j][k].value;
        }
      }
    }
    //std::cout<<i<<" "<<alpha;
    std::cout<<"max vote :"<<max_value<<std::endl;
    //return getMeanTransform(i,alpha);
    return accumulatorSpace[i][alpha].T_set[0];
  }

  decltype(auto) HypoVerification(const Eigen::Affine3f &T) {
    pcl::PointCloud<pcl::PointNormal>::Ptr temp =
        boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

    pcl::transformPointCloud(*this->model_cloud_with_normal, *temp, T);
    pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(
        new pcl::search::KdTree<pcl::PointNormal>());
    auto cnt = 0;
    double radius = 100;
    kdtree->setInputCloud(this->scene_cloud_with_normal);
    //#pragma omp parallel for shared(temp, radius, cnt,search_cloud, kdtree)
    // default(none) num_threads(15)
    for (auto i = temp->points.begin(); i != temp->points.end(); i++) {
      std::vector<int> indices;
      std::vector<float> distance;
      //#pragma omp critical
      kdtree->radiusSearch(*i, radius, indices, distance);
      if (!indices.empty()) {
        //#pragma omp critical
        cnt += 0;
        continue;
      } else {
        int num = 0;
        for (auto j = 0; j < indices.size(); ++j) {
          if (pcl::getAngle3D(
                  static_cast<const Eigen::Vector3f>(
                      scene_cloud_with_normal->points[indices[j]].normal),
                  static_cast<const Eigen::Vector3f>(i->normal), true) >= 25) {
            num++;
            break;
          } else {
            continue;
          }
        }
        if (num > 0) {
          //#pragma omp critical
          cnt++;
        } else {
          //#pragma omp critical
          cnt += 0;
        }
      }
    }

    //#pragma omp barrier
    return cnt;
  }


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
