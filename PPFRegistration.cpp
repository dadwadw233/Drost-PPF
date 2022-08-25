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
    //初始化accumulatorSpace
    accumulatorSpace.resize(this->scene_cloud_with_normal->size()+1);
    for(auto &s:accumulatorSpace){
      s.resize(static_cast<int>(360/(angle_discretization_step*180/M_PI))+1);
    }
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

#pragma omp parallel for shared(i, scene_reference_point_sampling_rate, \
                                R, cout, sum, cnt, T_) private(p1, p2, n1, n2, delta,       \
                                           data) default(none) num_threads(15)
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
              //std::cout<<"m_a:"<<model_data->second.angle*180/M_PI<<std::endl;
              //std::cout<<"s_a:"<<data.second.angle*180/M_PI<<std::endl;
              auto alpha = model_data->second.angle - data.second.angle >= 0
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
              model_d = this->model_trans->getData(Hash::Trans_key(model_data->second.r))->second.T.rotation() *model_d;
              Eigen::AngleAxisf ms(static_cast<float>(alpha), x);
              Eigen::Matrix4f Rx{};

              Rx<<ms.matrix()(0,0),ms.matrix()(0,1),ms.matrix()(0,2),0,
                  ms.matrix()(1,0),ms.matrix()(1,1),ms.matrix()(1,2),0,
                  ms.matrix()(2,0),ms.matrix()(2,1),ms.matrix()(2,2),0,
                  0,0,0,1;
              Eigen::Affine3f Rx_{Rx};
              //std::cout<<"first: \n"<<ms.matrix()<<std::endl;
              //std::cout<<"second: \n"<<Rx<<std::endl;
              //生成变换矩阵
              //Eigen::Matrix4f Tms = T_.inverse().matrix()*Rx_.matrix()*this->model_trans->getData(Hash::Trans_key(model_data->second.r))->second.T.matrix();
                Eigen::Matrix3f Rms = T_.rotation().inverse()*Rx_.rotation()*this->model_trans->getData(Hash::Trans_key(model_data->second.r))->second.T.rotation();
                Eigen::Vector3f tms = -T_.translation()+this->model_trans->getData(Hash::Trans_key(model_data->second.r))->second.T.translation();
                Eigen::Matrix4f Tms{};
                Tms<<Rms(0,0), Rms(0,1), Rms(0,2), tms[0],
                    Rms(1,0), Rms(1,1), Rms(1,2), tms[1],
                    Rms(2,0), Rms(2,1), Rms(2,2), tms[2],
                    0,0,0,1;
              Eigen::Affine3f Tms_(Tms);
              //验证变换矩阵正确性
/*
              //std::cout<<"before angle: \n"<<pcl::getAngle3D(model_d, scene_d,true)<<std::endl;
              //model_d = Tms_.rotation()*model_d;
              //model_d = Rx_.rotation()*this->model_trans->getData(Hash::Trans_key(model_data->second.r))->second.T.rotation()*model_d;

              //std::cout<<"after angle: \n"<<pcl::getAngle3D(model_d, scene_d,true)<<std::endl;


              //std::cout<<"m_a:"<<model_data->second.angle*180/M_PI<<std::endl;
              //std::cout<<"s_a:"<<data.second.angle*180/M_PI<<std::endl;
              //std::cout<<"a:"<<alpha*180/M_PI<<std::endl;
              model_d =
                  Eigen::Vector3f(
                      (model_data->second.t.x - model_data->second.r.x),
                      (model_data->second.t.y - model_data->second.r.y),
                      (model_data->second.t.z - model_data->second.r.z))
                      .normalized();
              scene_d =
                  Eigen::Vector3f((data.second.t.x - data.second.r.x),
                                  (data.second.t.y - data.second.r.y),
                                  (data.second.t.z - data.second.r.z))
                      .normalized();
              Eigen::Vector3f model_d_after = (Tms_.rotation() * model_d).normalized();
              //std::cout<<"after: \n"<<model_d_after<<std::endl;
              //std::cout<<"ref: \n"<<scene_d<<std::endl;
              //std::cout<<"before: \n"<<pcl::getAngle3D(model_d,scene_d,true)<<std::endl;
              //std::cout<<"after: \n"<<pcl::getAngle3D(model_d_after,scene_d,true)<<std::endl;
              Eigen::Vector3f model_n = (model_d.cross(Eigen::Vector3f (model_data->second.r.normal_x,model_data->second.r.normal_y, model_data->second.r.normal_z))).normalized();
              Eigen::Vector3f scene_n = (scene_d.cross(Eigen::Vector3f (data.second.r.normal_x,data.second.r.normal_y, data.second.r.normal_z))).normalized();

              float delta_ = acos(fminf(fmaxf(fabs(model_n.dot(scene_n))/(model_n.norm()*scene_n.norm()),-1.0),1.0))*180/M_PI;
              std::cout<<"before_delta_angle: \n"<<delta_<<std::endl;
              model_n = (model_d_after.cross(Tms_.rotation()*Eigen::Vector3f (model_data->second.r.normal_x,model_data->second.r.normal_y, model_data->second.r.normal_z))).normalized();
              scene_n = (scene_d.cross(Eigen::Vector3f (data.second.r.normal_x,data.second.r.normal_y, data.second.r.normal_z))).normalized();

              delta_ = acos(fminf(fmaxf(fabs(model_n.dot(scene_n))/(model_n.norm()*scene_n.norm()),-1.0),1.0))*180/M_PI;
              std::cout<<"after_delta_angle: \n"<<delta_<<std::endl;
*/
              //离散化alpha

              alpha = static_cast<int>(std::floor(180*alpha/M_PI/(angle_discretization_step*180/M_PI)));
              //std::cout<<alpha<<std::endl;
#pragma omp critical
              vote(i, alpha,T_);


              ++model_data;
            }
          }
        }
      }

    }

  }
}
void PPFRegistration::vote(const int& i_, const int& alpha_,
                           const Eigen::Affine3f& T_) {

  accumulatorSpace[i_][alpha_].value+=1;

  accumulatorSpace[i_][alpha_].T_set.push_back(T_);
}
}  // namespace PPF