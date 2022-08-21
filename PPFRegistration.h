//
// Created by yyh on 22-8-18.
//

#ifndef DROST_PPF_PPFREGISTRATION_H
#define DROST_PPF_PPFREGISTRATION_H
#include "Hashmap.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "omp.h"
namespace PPF {

class PPFRegistration {
 public:
  PPFRegistration() = default;

  void compute(const pcl::PointCloud<pcl::PointNormal>::Ptr &scene, const Hash::HashMap::Ptr &search_map);

 private:

};

}  // namespace PPF

#endif  // DROST_PPF_PPFREGISTRATION_H
