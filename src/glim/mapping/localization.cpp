#include <glim/mapping/localization.hpp>

#include <unordered_set>
#include <spdlog/spdlog.h>
#include <boost/filesystem.hpp>

#include <gtsam/base/serialization.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/rotate_vector3_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/isam2_ext_dummy.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>

#include <glim/util/config.hpp>
#include <glim/util/serialization.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/mapping/callbacks.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::M;

using Callbacks = LocalizationCallbacks;

LocalizationParams::LocalizationParams(): GlobalMappingParams() {
}

LocalizationParams::~LocalizationParams() {}

Localization::Localization(const LocalizationParams& params) : GlobalMapping(params), params(params) {
}

Localization::~Localization() {}

  // boost::shared_ptr<gtsam::NonlinearFactorGraph> create_relocalization_factors(const SubMap::Ptr& submap, const Eigen::Isometry3d& initial_pose,

boost::shared_ptr<gtsam::NonlinearFactorGraph> Localization::create_relocalization_factors(
  const SubMap::Ptr& submap, const Eigen::Isometry3d& initial_pose,
  double linear_search_window, double angular_search_window) {

  // generate rotated pointcloud and matching candidates

  // for each candidates, compute overlap score



  // ICP align with the best candidate
  auto factors = gtsam::make_shared<gtsam::NonlinearFactorGraph>();


  relocalization = false;

  double min_distance = params.max_implicit_loop_distance;
  glim::SubMap::Ptr prebuilt_map{nullptr};

  for (int i = 0; i < prebuilt_submaps.size(); i++) {
    const double dist = (prebuilt_submaps[i]->T_world_origin.translation() - submap->T_world_origin.translation()).norm();
    if (dist >= min_distance) {
      continue;
    }
    else {
      min_distance = dist;
      prebuilt_map = prebuilt_submaps[i];
    }
  }

  if (!prebuilt_map)
    return factors;


  // example code of align, assume using one submap
  // auto & prebuilt_map = prebuilt_submaps.front();
  double best_overlap_score = 0.0;
  auto best_initial_pose = find_best_candidate(prebuilt_map, submap, linear_search_window,
    angular_search_window, initial_pose, best_overlap_score);

  if (best_overlap_score < params.min_implicit_loop_overlap)
    return factors;

  // const int last = current - 1;
  const gtsam::Pose3 init_delta = gtsam::Pose3(best_initial_pose.matrix());


  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3::Identity());
  values.insert(X(1), init_delta);

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(X(0), X(1), prebuilt_map->frame, submap->frame);
  factor->set_max_correspondence_distance(0.5);
  factor->set_num_threads(2);
  graph.add(factor);

  logger->debug("--- LM optimization ---");
  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setlambdaInitial(1e-12);
  lm_params.setMaxIterations(10);
  lm_params.callback = [this](const auto& status, const auto& values) { logger->debug(status.to_string()); };

#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
  arena->execute([&] {
#endif
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    values = optimizer.optimize();

#ifdef GTSAM_USE_TBB
  });
#endif

  const gtsam::Pose3 estimated_delta = values.at<gtsam::Pose3>(X(1));
  const auto linearized = factor->linearize(values);
  const auto H = linearized->hessianBlockDiagonal()[X(1)] + 1e6 * gtsam::Matrix6::Identity();

  factors->add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(M(prebuilt_map->id), X(submap->id), estimated_delta, gtsam::noiseModel::Gaussian::Information(H)));
  return factors;
  // add between factor

  // save to T_map_odom

  // add relocalization factor
  // new_factors->add(*factors);

  // Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  // try {
  //   auto result = update_isam2(*new_factors, *new_values);
  //   Callbacks::on_smoother_update_result(*isam2, result);
  // } catch (std::exception& e) {
  //   logger->error("an exception was caught during global map optimization!!");
  //   logger->error(e.what());
  // }
  // new_values.reset(new gtsam::Values);
  // new_factors.reset(new gtsam::NonlinearFactorGraph);

  // update_submaps();
  // Callbacks::on_update_submaps(submaps);
}

Eigen::Isometry3d Localization::find_best_candidate(
    const SubMap::Ptr& target_map, const SubMap::Ptr& submap,
    double linear_search_window, double angular_search_window,
    Eigen::Isometry3d initial_pose, double& best_overlap_score) const
{
  double linear_step = 0.2;
  double angular_step = 10 / 180 * M_PI;

  Eigen::Matrix3d rot_mat = initial_pose.rotation();
  Eigen::Vector3d euler = rot_mat.eulerAngles(0, 1, 2);
  Eigen::Vector3d translation = initial_pose.translation();

  best_overlap_score  = 0.0;
  Eigen::Isometry3d best_initial_pose;

  for (double delta_x = -linear_search_window/2.0; delta_x < linear_search_window; delta_x += linear_step)
    for (double delta_y = -linear_search_window/2.0; delta_y < linear_search_window; delta_y += linear_step)
      for (double delta_theta = -angular_search_window/2.0; delta_theta < angular_search_window; delta_theta += angular_step) {

          Eigen::Vector3d new_translation = Eigen::Vector3d(translation.x() + delta_x,
                                              translation.y() + delta_y, translation.z());

          Eigen::Matrix3d new_rotation;
          new_rotation = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
              * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(euler[2] + delta_theta, Eigen::Vector3d::UnitZ());

          Eigen::Isometry3d new_pose;
          new_pose.translation() = new_translation;
          new_pose.linear() = new_rotation;

          double overlap_score  = gtsam_points::overlap_auto(target_map->voxelmaps.back(), submap->frame, new_pose);
          if (overlap_score < params.min_implicit_loop_overlap) {
            continue;
          }

          if (overlap_score > best_overlap_score) {
            best_overlap_score = overlap_score;
            best_initial_pose = new_pose;
          }
      }

  return best_initial_pose;
}

// may be add matching cost function to map

void Localization::insert_submap(const SubMap::Ptr& submap) {
  logger->debug("insert_submap id={} |frame|={}", submap->id, submap->frame->size());

  if (relocalize_submap->id == submap->id && relocalization) {
    auto relocalization_factors = create_relocalization_factors(relocalize_submap, initial_pose_, 6, 2 * M_PI);
    if (relocalization_factors->size() > 0) {
      new_factors->add(*relocalization_factors);
      relocalized = true;
    }
  }

  const int current = submaps.size();
  const int last = current - 1;
  GlobalMapping::insert_submap(current, submap);

  gtsam::Pose3 current_T_world_submap = gtsam::Pose3::Identity();
  gtsam::Pose3 last_T_world_submap = gtsam::Pose3::Identity();

  // trim submap if > number of submap keep

  if (current != 0) {
    if (isam2->valueExists(X(last))) {
      last_T_world_submap = isam2->calculateEstimate<gtsam::Pose3>(X(last));
    } else {
      last_T_world_submap = new_values->at<gtsam::Pose3>(X(last));
    }

    const Eigen::Isometry3d T_origin0_endpointR0 = submaps[last]->T_origin_endpoint_R;
    const Eigen::Isometry3d T_origin1_endpointL1 = submaps[current]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_endpointR0_endpointL1 = submaps[last]->odom_frames.back()->T_world_sensor().inverse() * submaps[current]->odom_frames.front()->T_world_sensor();
    const Eigen::Isometry3d T_origin0_origin1 = T_origin0_endpointR0 * T_endpointR0_endpointL1 * T_origin1_endpointL1.inverse();

    current_T_world_submap = last_T_world_submap * gtsam::Pose3(T_origin0_origin1.matrix());
  } else {
    current_T_world_submap = gtsam::Pose3(submap->T_world_origin.matrix());
  }

  new_values->insert(X(current), current_T_world_submap);
  submap->T_world_origin = Eigen::Isometry3d(current_T_world_submap.matrix());

  Callbacks::on_insert_submap(submap);

  submap->drop_frame_points();

  if (current == 0) {
    new_factors->emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params.init_pose_damping_scale);
  } else {
    new_factors->add(*create_between_factors(current));

    // matching_cost_factors: may be not needed in localization mode
    // new_factors->add(*create_map_matching_cost_factors(current));

    // create localization factor
    if (relocalized)
      new_factors->add(*create_map_matching_cost_factors(current));
  }

  if (params.enable_imu) {
    if (submap->odom_frames.front()->frame_id != FrameID::IMU) {
      logger->warn("odom frames are not estimated in the IMU frame while global mapping requires IMU estimation");
    }

    // Local velocities
    const gtsam::imuBias::ConstantBias imu_biasL(submap->frames.front()->imu_bias);
    const gtsam::imuBias::ConstantBias imu_biasR(submap->frames.back()->imu_bias);

    const Eigen::Vector3d v_origin_imuL = submap->T_world_origin.linear().inverse() * submap->frames.front()->v_world_imu;
    const Eigen::Vector3d v_origin_imuR = submap->T_world_origin.linear().inverse() * submap->frames.back()->v_world_imu;

    const auto prior_noise3 = gtsam::noiseModel::Isotropic::Precision(3, 1e6);
    const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);

    if (current > 0) {
      new_values->insert(E(current * 2), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_L).matrix()));
      new_values->insert(V(current * 2), (submap->T_world_origin.linear() * v_origin_imuL).eval());
      new_values->insert(B(current * 2), imu_biasL);

      new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2), gtsam::Pose3(submap->T_origin_endpoint_L.matrix()), prior_noise6);
      new_factors->emplace_shared<gtsam_points::RotateVector3Factor>(X(current), V(current * 2), v_origin_imuL, prior_noise3);
      new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), imu_biasL, prior_noise6);
      new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), B(current * 2 + 1), gtsam::imuBias::ConstantBias(), prior_noise6);
    }

    new_values->insert(E(current * 2 + 1), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_R).matrix()));
    new_values->insert(V(current * 2 + 1), (submap->T_world_origin.linear() * v_origin_imuR).eval());
    new_values->insert(B(current * 2 + 1), imu_biasR);

    new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2 + 1), gtsam::Pose3(submap->T_origin_endpoint_R.matrix()), prior_noise6);
    new_factors->emplace_shared<gtsam_points::RotateVector3Factor>(X(current), V(current * 2 + 1), v_origin_imuR, prior_noise3);
    new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2 + 1), imu_biasR, prior_noise6);

    if (current != 0) {
      const double stampL = submaps[last]->frames.back()->stamp;
      const double stampR = submaps[current]->frames.front()->stamp;

      int num_integrated;
      const int imu_read_cursor = imu_integration->integrate_imu(stampL, stampR, imu_biasL, &num_integrated);
      imu_integration->erase_imu_data(imu_read_cursor);

      if (num_integrated < 2) {
        logger->warn("insufficient IMU data between submaps (global_mapping)!!");
        new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(V(last * 2 + 1), V(current * 2), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1.0));
      } else {
        new_factors
          ->emplace_shared<gtsam::ImuFactor>(E(last * 2 + 1), V(last * 2 + 1), E(current * 2), V(current * 2), B(last * 2 + 1), imu_integration->integrated_measurements());
      }
    }
  }

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  try {
    auto result = update_isam2(*new_factors, *new_values);
    Callbacks::on_smoother_update_result(*isam2, result);
  } catch (std::exception& e) {
    logger->error("an exception was caught during global map optimization!!");
    logger->error(e.what());
  }
  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

bool Localization::load(const std::string& path) {
  std::ifstream ifs(path + "/graph.txt");
  if (!ifs) {
    logger->error("failed to open {}/graph.txt", path);
    return false;
  }

  std::string token;
  int num_submaps, num_all_frames, num_matching_cost_factors;

  ifs >> token >> num_submaps;
  ifs >> token >> num_all_frames;
  ifs >> token >> num_matching_cost_factors;

  std::vector<std::tuple<std::string, int, int>> matching_cost_factors(num_matching_cost_factors);
  for (int i = 0; i < num_matching_cost_factors; i++) {
    auto& factor = matching_cost_factors[i];
    ifs >> token >> std::get<0>(factor) >> std::get<1>(factor) >> std::get<2>(factor);
  }

  logger->info("Load submaps");
  prebuilt_submaps.resize(num_submaps);
  prebuilt_subsampled_submaps.resize(num_submaps);
  for (int i = 0; i < num_submaps; i++) {
    auto submap = SubMap::load((boost::format("%s/%06d") % path % i).str());
    if (!submap) {
      return false;
    }

    gtsam_points::PointCloud::Ptr subsampled_submap = gtsam_points::random_sampling(submap->frame, params.randomsampling_rate, mt);

    prebuilt_submaps[i] = submap;
    prebuilt_submaps[i]->voxelmaps.clear();
    prebuilt_subsampled_submaps[i] = subsampled_submap;

    if (params.enable_gpu) {
#ifdef BUILD_GTSAM_POINTS_GPU
      prebuilt_subsampled_submaps[i] = gtsam_points::PointCloudGPU::clone(*prebuilt_subsampled_submaps[i]);

      for (int j = 0; j < params.submap_voxelmap_levels; j++) {
        const double resolution = params.submap_voxel_resolution * std::pow(params.submap_voxelmap_scaling_factor, j);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
        voxelmap->insert(*prebuilt_subsampled_submaps[i]);
        prebuilt_submaps[i]->voxelmaps.push_back(voxelmap);
      }
#else
      logger->warn("GPU is enabled for global_mapping but gtsam_points was built without CUDA!!");
#endif
    } else {
      for (int j = 0; j < params.submap_voxelmap_levels; j++) {
        const double resolution = params.submap_voxel_resolution * std::pow(params.submap_voxelmap_scaling_factor, j);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
        voxelmap->insert(*prebuilt_subsampled_submaps[i]);
        prebuilt_submaps[i]->voxelmaps.push_back(voxelmap);
      }
    }

    Callbacks::on_insert_localization_submap(submap);
    std::vector<SubMap::Ptr> submaps_{submap};
    Callbacks::on_update_localization_submaps(submaps_);

  }

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;

  logger->info("creating prior factors");
  for (const auto &submap: prebuilt_submaps) {
    values.insert(M(submap->id), gtsam::Pose3(submap->T_world_origin.matrix()));
    // graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(M(submap->id), gtsam::Pose3(submap->T_world_origin.matrix()),
    //   gtsam::noiseModel::Isotropic::Precision(6, 1e1));
    graph.emplace_shared<gtsam::NonlinearEquality1<gtsam::Pose3>>(gtsam::Pose3(submap->T_world_origin.matrix()), M(submap->id));
  }
  logger->info("done");



  return true;
}


boost::shared_ptr<gtsam::NonlinearFactorGraph> Localization::create_map_matching_cost_factors(int current) const {
  auto factors = gtsam::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0) {
    return factors;
  }

  const auto& current_submap = submaps.back();

  for (int i = 0; i < prebuilt_submaps.size(); i++) {
    const double dist = (prebuilt_submaps[i]->T_world_origin.translation() - current_submap->T_world_origin.translation()).norm();
    if (dist > params.max_implicit_loop_distance) {
      continue;
    }

    const Eigen::Isometry3d delta = prebuilt_submaps[i]->T_world_origin.inverse() * current_submap->T_world_origin;
    const double overlap = gtsam_points::overlap_auto(prebuilt_submaps[i]->voxelmaps.back(), current_submap->frame, delta);

    if (overlap < params.min_implicit_loop_overlap) {
      continue;
    }

    if (params.registration_error_factor_type == "VGICP") {
      for (const auto& voxelmap : prebuilt_submaps[i]->voxelmaps) {
        factors->emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(i), X(current), voxelmap, subsampled_submaps[current]);
      }
    }
#ifdef BUILD_GTSAM_POINTS_GPU
    else if (params.registration_error_factor_type == "VGICP_GPU") {
      const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
      const auto& stream = stream_buffer.first;
      const auto& buffer = stream_buffer.second;
      for (const auto& voxelmap : submaps[i]->voxelmaps) {
        factors->emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(i), X(current), voxelmap, subsampled_submaps[current], stream, buffer);
      }
    }
#endif
    else {
      logger->warn("unknown registration error type ({})", params.registration_error_factor_type);
    }
  }

  return factors;
}

}  // namespace glim
