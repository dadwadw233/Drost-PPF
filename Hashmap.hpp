//
// Created by yyh on 22-8-18.
//

#ifndef DROST_PPF_HASHMAP_HPP
#define DROST_PPF_HASHMAP_HPP

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <unordered_map>
#include <utility>
#include <vector>
#include "Eigen/Core"
#include "pcl/console/print.h"
#include "pcl/point_cloud.h"
namespace PPF {
namespace Hash {

struct HashData {
  pcl::PointNormal r;
  pcl::PointNormal t;
  double angle{};
  HashData(const pcl::PointNormal &r_, const pcl::PointNormal &t_,
           const double angle_)
      : r(r_), t(t_), angle(angle_){};
  HashData() = default;
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
struct hash_cal {
  size_t operator()(Hash::HashKey k) const {
    return std::hash<int>()(k.k1) ^ (std::hash<int>()(k.k2) << 1) ^
           (std::hash<int>()(k.k3) << 2) ^ (std::hash<int>()(k.k4) << 3) ^
           (std::hash<int>()(k.k1) << 4) ^ (std::hash<int>()(k.k2) << 5) ^
           (std::hash<int>()(k.k3) << 5) ^ (std::hash<int>()(k.k4) << 6);
    // return std::hash<int>()(k.k1);
  }
};
struct Trans_data {
  Eigen::Affine3f T;
  Trans_data() = default;
  explicit Trans_data(const Eigen::Affine3f &T_) : T(T_){};
};
struct Trans_key {
  pcl::PointNormal p;
  bool operator==(const Trans_key &k) const {
    return p.x == k.p.x && p.y == k.p.y && p.z == k.p.z;
  }
  explicit Trans_key(const pcl::PointNormal &p_) : p(p_){};
  Trans_key() = default;
};
struct Tran_cal {
  size_t operator()(Hash::Trans_key k) const {
    return std::hash<int>()(k.p.x) ^ std::hash<int>()(k.p.y) << 1 ^
           std::hash<int>()(k.p.z) << 2;
    // return std::hash<int>()(k.k1);
  }
};

template <class hash_key, class hash_data, class hash_cal_>
class HashMap_ {
 public:
  using Ptr = boost::shared_ptr<HashMap_<hash_key, hash_data, hash_cal_>>;
  bool addInfo(hash_key &&key, hash_data &&data);

  bool addInfo(hash_key &key, hash_data &data);

  bool addInfo(std::pair<hash_key, hash_data> &&data);

  bool addInfo(std::pair<hash_key, hash_data> &data);

  decltype(auto) getData(hash_key & key) { return (this->map.find(key)); }
  decltype(auto) getData(hash_key && key) { return (this->map.find(key)); }
  decltype(auto) begin() { return this->map.begin(); }

  decltype(auto) getSameKeyNum(hash_key &key) { return this->map.count(key); }

  bool find(hash_key &key);
  bool find(hash_key &&key);

  bool empty() { return this->map.empty(); }

 public:
  using iterator = typename std::unordered_multimap<hash_key, hash_data,
                                                    hash_cal_>::iterator;

 private:
  std::unordered_multimap<hash_key, hash_data, hash_cal_> map;
};
template <class hash_key, class hash_data, class hash_cal_>
bool HashMap_<hash_key, hash_data, hash_cal_>::find(hash_key &key) {
  if (map.find(key) != map.end()) {
    return true;
  } else {
    return false;
  }
}
template <class hash_key, class hash_data, class hash_cal_>
bool HashMap_<hash_key, hash_data, hash_cal_>::addInfo(
    std::pair<hash_key, hash_data> &&data) {
  try {
    this->map.insert(data);
    return true;
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    return false;
  }
}
template <class hash_key, class hash_data, class hash_cal_>
bool HashMap_<hash_key, hash_data, hash_cal_>::addInfo(
    std::pair<hash_key, hash_data> &data) {
  try {
    this->map.insert(data);
    return true;
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    return false;
  }
}
template <class hash_key, class hash_data, class hash_cal_>
bool HashMap_<hash_key, hash_data, hash_cal_>::addInfo(hash_key&& key,
                                                       hash_data&& data) {
  try {
    this->map.emplace(key, data);
    return true;
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    return false;
  }
}
template <class hash_key, class hash_data, class hash_cal_>
bool HashMap_<hash_key, hash_data, hash_cal_>::addInfo(hash_key& key,
                                                       hash_data& data) {
  try {
    this->map.emplace(key, data);
    return true;
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    return false;
  }
}
};  // namespace Hash
}  // namespace PPF
#endif  // DROST_PPF_HASHMAP_HPP
