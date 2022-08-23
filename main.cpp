//
// Created by yyh on 22-8-18.
//
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include <pcl/console/parse.h>
#include "Drost_PPF.h"
int main(int argc, char** argv) {
  if (argc <= 1) {
    PCL_ERROR("Syntax: ./Drost_PPF pcd_model_list pcd_scene(optional)\n");
    return -1;
  }
  std::vector<int> pcd_file_indices =
      pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  if (pcd_file_indices.size() < 1) {
    PCL_ERROR("need pcd file as input\n");
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDReader reader;
  reader.read(argv[1], *model);
  reader.read(argv[2], *scene);
  PPF::Drost_PPF handle(model,scene);
  handle.test();

  return 0;
}
