//
// Created by yyh on 22-8-18.
//

#include "Hashmap.h"
namespace PPF {
namespace Hash {

bool HashMap::addInfo(Hash::HashKey &key, Hash::HashData &data) {
  try {
    this->map.emplace(key, data);
    return true;
  } catch (std::bad_alloc) {
    PCL_INFO("bad alloc !! ");
    return false;
  }
}
bool HashMap::addInfo(std::pair<Hash::HashKey, Hash::HashData> &data) {
  try {
    this->map.insert(data);
    return true;
  } catch (std::bad_alloc) {
    PCL_INFO("bad alloc !! ");
    return false;
  }
}

bool HashMap::find(Hash::HashKey &key) {
  if (map.find(key) != map.end()) {
    return true;
  } else {
    return false;
  }
}




}  // namespace Hash
}  // namespace PPF
