//
// Created by yyh on 22-8-18.
//

#ifndef DROST_PPF_HASHMAP_H
#define DROST_PPF_HASHMAP_H

#include <pcl/features/normal_3d.h>
#include <pcl/features/ppf.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#include <fstream>
#include <limits>
#include <pcl/search/impl/search.hpp>
#include <unordered_map>
#include <utility>
#include <vector>
#include "Eigen/Core"
#include "pcl/console/print.h"
#include "pcl/point_cloud.h"
#include "pcl/point_representation.h"
#include "pcl/registration/registration.h"
#include "pcl/visualization/cloud_viewer.h"
namespace PPF {
namespace Hash {
struct HashData {
  pcl::PointNormal r;
  pcl::PointNormal t;
};
struct HashKey {
  int k1;
  int k2;
  int k3;
  int k4;
  bool operator==(const HashKey &k) const {
    return k1 == k.k1 && k2 == k.k2 && k3 == k.k3 && k4 == k.k4;
  }
};

class HashMap {
 public:
  bool addInfo(Hash::HashKey &key, Hash::HashData &data);

  bool addInfo(std::pair<Hash::HashKey, Hash::HashData> &data);

  HashData getData(Hash::HashKey &key);

  bool find(Hash::HashKey &key);

  decltype(auto) begin();

  bool empty();

  HashMap() {}

 private:
  struct hash_cal {
    size_t operator()(const HashKey &k) const {
      return std::hash<int>()(k.k1) ^ (std::hash<int>()(k.k2) << 1) ^
             (std::hash<int>()(k.k3) << 2) ^ (std::hash<int>()(k.k4) << 3) ^
             (std::hash<int>()(k.k1) << 4) ^ (std::hash<int>()(k.k2) << 5) ^
             (std::hash<int>()(k.k3) << 5) ^ (std::hash<int>()(k.k4) << 6);
      // return std::hash<int>()(k.k1);
    }
  };
  std::unordered_multimap<HashKey, HashData, hash_cal> map;
};
typedef boost::shared_ptr<Hash::HashMap> Ptr;

}  // namespace Hash
}  // namespace PPF
#endif  // DROST_PPF_HASHMAP_H
