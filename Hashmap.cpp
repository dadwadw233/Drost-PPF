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
  } catch (std::exception &e) {
    std::cout<<e.what()<<std::endl;
    return false;
  }
}
bool HashMap::addInfo(std::pair<Hash::HashKey, Hash::HashData> &data) {
  try {
    this->map.insert(data);
    return true;
  } catch (std::exception &e) {
    std::cout<<e.what()<<std::endl;
    return false;
  }
}

HashData HashMap::getData(Hash::HashKey &key) {
  return (*this->map.find(key)).second;
}

bool HashMap::find(Hash::HashKey &key) {
  if (map.find(key) != map.end()) {
    return true;
  } else {
    return false;
  }
}

bool HashMap::empty() { return this->map.empty(); }

decltype(auto) HashMap::begin() { return this->map.begin(); }
}  // namespace Hash
}  // namespace PPF
