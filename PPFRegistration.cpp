//
// Created by yyh on 22-8-18.
//

#include "PPFRegistration.h"
#include <pcl/common/common.h>

namespace PPF {
Eigen::Affine3f PPFRegistration::getFinalTransformation() {
  return this->finalTransformation;
}
void PPFRegistration::setSceneReferencePointSamplingRate(
    const float& scene_reference_point_sampling_rate) {
  this->scene_reference_point_sampling_rate =
      scene_reference_point_sampling_rate;
}
void PPFRegistration::setPositionClusteringThreshold(
    const float& clustering_position_diff_threshold) {
  this->clustering_position_diff_threshold = clustering_position_diff_threshold;
}
void PPFRegistration::setRotationClusteringThreshold(
    const float& clustering_rotation_diff_threshold) {
  this->clustering_rotation_diff_threshold = clustering_rotation_diff_threshold;
}
void PPFRegistration::setDiscretizationSteps(
    const float& angle_discretization_step,
    const float& distance_discretization_step) {
  this->angle_discretization_step = angle_discretization_step;
  this->distance_discretization_step = distance_discretization_step;
}

void PPFRegistration::setDobj(const float& data) { this->d_obj = data; }

void PPFRegistration::setSearchMap(
    const Hash::HashMap_<Hash::HashKey, Hash::HashData, Hash::hash_cal>::Ptr&
        searchMap) {
  this->searchMap = searchMap;
}
void PPFRegistration::setTransMap(
    const Hash::HashMap_<Hash::Trans_key, Hash::Trans_data,
                         Hash::Tran_cal>::Ptr& Trans) {
  this->model_trans = Trans;
}
void PPFRegistration::setInputSource(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) {
  this->model_cloud_with_normal = cloud;
}
void PPFRegistration::setInputTarget(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) {
  this->scene_cloud_with_normal = cloud;
}

PPFRegistration::PPFRegistration() {
  this->model_cloud_with_normal =
      boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  this->scene_cloud_with_normal =
      boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  this->searchMap = boost::make_shared<
      Hash::HashMap_<Hash::HashKey, Hash::HashData, Hash::hash_cal>>();
  this->model_trans = boost::make_shared<
      Hash::HashMap_<Hash::Trans_key, Hash::Trans_data, Hash::Tran_cal>>();
}
bool PPFRegistration::check() {
  if (this->model_cloud_with_normal->points.empty() ||
      this->scene_cloud_with_normal->points.empty() ||
      this->searchMap->empty() || this->model_trans->empty()) {
    PCL_ERROR("Neither point cloud or search map/trans map should be empty!\n");
    PCL_ERROR("Init failed, make sure all params are initialized and retry\n");
    return false;
  } else if (!scene_reference_point_sampling_rate ||
             !clustering_position_diff_threshold ||
             !clustering_rotation_diff_threshold) {
    PCL_ERROR("Params for clustering are not be initialized\n");
    PCL_ERROR("Init failed, make sure all params are initialized and retry\n");
    return false;
  } else if (!angle_discretization_step || !distance_discretization_step) {
    PCL_ERROR("Params for PPF establishment are not be initialized\n");
    PCL_ERROR("Init failed, make sure all params are initialized and retry\n");
    return false;
  } else {
    PCL_INFO("Pass init check");
    return true;
  }
}
void PPFRegistration::compute() {
  if (!check()) {
    PCL_ERROR("Initialization check failed, online processing terminated");
    return;
  } else {
    auto tp1 = boost::chrono::steady_clock::now();
    std::pair<Hash::HashKey, Hash::HashData> data{};
    Eigen::Vector3f p1{};
    Eigen::Vector3f p2{};
    Eigen::Vector3f n1{};
    Eigen::Vector3f n2{};
    Eigen::Vector3f delta{};
    std::cout << "finish Registering init" << std::endl;
    std::cout << "computing ..." << std::endl;
    int cnt = 0;
    double sum = 0;
    for (auto i = 0; i < scene_cloud_with_normal->points.size(); ++i) {
      Eigen::Vector3f x_n{1, 0, 0};
      auto ref_alpha = pcl::getAngle3D(
          Eigen::Vector3f(scene_cloud_with_normal->points[i].normal_x,
                          scene_cloud_with_normal->points[i].normal_y,
                          scene_cloud_with_normal->points[i].normal_z),
          x_n);
      Eigen::Vector3f t{-scene_cloud_with_normal->points[i].x,
                        -scene_cloud_with_normal->points[i].y,
                        -scene_cloud_with_normal->points[i]
                             .z};  // transition between mr and O
      Eigen::Vector3f n_ =
          (Eigen::Vector3f(scene_cloud_with_normal->points[i].normal_x,
                           scene_cloud_with_normal->points[i].normal_y,
                           scene_cloud_with_normal->points[i].normal_z)
               .cross(x_n))
              .normalized();
      Eigen::AngleAxisf v(static_cast<float>(ref_alpha), n_);

      Eigen::Matrix3f R;
      R << v.matrix();
      Eigen::Matrix4f T;
      T << R(0, 0), R(0, 1), R(0, 2), t[0], R(1, 0), R(1, 1), R(1, 2), t[1],
          R(2, 0), R(2, 1), R(2, 2), t[2], 0, 0, 0, 1;
      Eigen::Affine3f T_(T);
      model_trans->addInfo(
          PPF::Hash::Trans_key(scene_cloud_with_normal->points[i]),
          PPF::Hash::Trans_data(T_));
/*#pragma omp parallel for shared(i, scene_reference_point_sampling_rate, \
                                R, cout, sum, cnt) private(p1, p2, n1, n2, delta,       \
                                           data) default(none) num_threads(15)*/
      for (auto j = 0; j < scene_cloud_with_normal->points.size(); j++) {
        if (i == j) {
          continue;
        } else {
          p1 << scene_cloud_with_normal->points[i].x,
              scene_cloud_with_normal->points[i].y,
              scene_cloud_with_normal->points[i].z;
          p2 << scene_cloud_with_normal->points[j].x,
              scene_cloud_with_normal->points[j].y,
              scene_cloud_with_normal->points[j].z;
          n1 << scene_cloud_with_normal->points[i].normal_x,
              scene_cloud_with_normal->points[i].normal_y,
              scene_cloud_with_normal->points[i].normal_z;
          n2 << scene_cloud_with_normal->points[j].normal_x,
              scene_cloud_with_normal->points[j].normal_y,
              scene_cloud_with_normal->points[j].normal_z;

          delta = p2 - p1;
          float f4 = delta.norm();
          Eigen::Vector3f d = delta.normalized();
          d = R * d;
          Eigen::Vector3f x{1, 0, 0};
          Eigen::Vector3f z{0, 0, 1};
          Eigen::Vector3f y{0, 1, 0};

          double scene_alpha =
              acos(fabs(d.cross(x).dot(y)) / (d.cross(x).norm() * y.norm()));

          if (fabs(pcl::getAngle3D(d, z, true)) <= 90 &&
              fabs(pcl::getAngle3D(d, y, true)) <= 90) {
            scene_alpha = scene_alpha;
          } else if (fabs(pcl::getAngle3D(d, z, true)) <= 90 &&
                     fabs(pcl::getAngle3D(d, y, true)) >= 90) {
            scene_alpha = 2 * M_PI - scene_alpha;
          } else if (fabs(pcl::getAngle3D(d, z, true)) >= 90 &&
                     fabs(pcl::getAngle3D(d, y, true)) >= 90) {
            scene_alpha = M_PI + scene_alpha;
          } else {
            scene_alpha = M_PI - scene_alpha;
          }
          data.second.angle = scene_alpha;
          delta /= f4;

          float f1 = n1[0] * delta[0] + n1[1] * delta[1] + n1[2] * delta[2];

          float f2 = n1[0] * delta[0] + n2[1] * delta[1] + n2[2] * delta[2];

          float f3 = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];

          data.first.k1 =
              static_cast<int>(std::floor(f1 / angle_discretization_step));
          data.first.k2 =
              static_cast<int>(std::floor(f2 / angle_discretization_step));
          data.first.k3 =
              static_cast<int>(std::floor(f3 / angle_discretization_step));
          data.first.k4 =
              static_cast<int>(std::floor(f4 / distance_discretization_step));

          data.second.r = scene_cloud_with_normal->points[i];
          data.second.t = scene_cloud_with_normal->points[j];
          if (searchMap->find(data.first)) {
            auto model_data = this->searchMap->getData(data.first);
            auto same_key_num = this->searchMap->getSameKeyNum(data.first);

            for (auto k = 0; k < same_key_num; ++k) {
              auto alpha = model_data->second.angle - data.second.angle > 0
                               ? model_data->second.angle - data.second.angle
                               : 2*M_PI-data.second.angle + model_data->second.angle;

              auto model_d =
                  Eigen::Vector3f(
                      (model_data->second.t.x - model_data->second.r.x),
                      (model_data->second.t.y - model_data->second.r.y),
                      (model_data->second.t.z - model_data->second.r.z))
                      .normalized();
              auto scene_d =
                  Eigen::Vector3f((data.second.t.x - data.second.r.x),
                                  (data.second.t.y - data.second.r.y),
                                  (data.second.t.z - data.second.r.z))
                      .normalized();
              scene_d = R * scene_d;
              model_d = this->model_trans
                            ->getData(Hash::Trans_key(model_data->second.r))
                            ->second.T.rotation() *
                        model_d;

              Eigen::AngleAxisf ms(static_cast<float>(alpha), x);

              Eigen::Matrix3f msR;
              msR<< ms.matrix();
              Eigen::Matrix4f msT;
              Eigen::Vector3f model_d_after = msR * model_d;
              //std::cout<<"after: \n"<<model_d_after<<std::endl;
              //std::cout<<"ref: \n"<<scene_d<<std::endl;
              //std::cout<<"before: \n"<<pcl::getAngle3D(model_d,scene_d,true)<<std::endl;
              //std::cout<<"after: \n"<<pcl::getAngle3D(model_d_after,scene_d,true)<<std::endl;
              Eigen::Vector3f model_n = (model_d.cross(x)).normalized();
              Eigen::Vector3f scene_n = (scene_d.cross(x)).normalized();

              float delta_ = acos(fminf(fmaxf(fabs(model_n.dot(scene_n))/(model_n.norm()*scene_n.norm()),-1.0),1.0))*180/M_PI;
              //std::cout<<"before_delta_angle: \n"<<delta_<<std::endl;
              model_n = (model_d_after.cross(x)).normalized();
              scene_n = (scene_d.cross(x)).normalized();

              //delta_ = acos(fminf(fmaxf(fabs(model_n.dot(scene_n))/(model_n.norm()*scene_n.norm()),-1.0),1.0))*180/M_PI;
              //std::cout<<"after_delta_angle: \n"<<delta_<<std::endl;
#pragma omp critical
              sum+=delta_;
#pragma omp critical
              ++cnt;
              model_data++;
            }
          }
        }
      }

    }
    std::cout<<"average: \n"<<sum/cnt<<std::endl;
  }
}
}  // namespace PPF