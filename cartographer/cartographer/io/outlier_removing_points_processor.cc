/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/io/outlier_removing_points_processor.h"

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

std::unique_ptr<OutlierRemovingPointsProcessor>
OutlierRemovingPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  const double miss_per_hit_limit = [&]() {
    if (!dictionary->HasKey("miss_per_hit_limit")) {
      LOG(INFO) << "Using default value of 3 for miss_per_hit_limit.";
      return 3.;
    } else {
      return dictionary->GetDouble("miss_per_hit_limit");
    }
  }();
  return absl::make_unique<OutlierRemovingPointsProcessor>(
      dictionary->GetDouble("voxel_size"), miss_per_hit_limit, next);
}

OutlierRemovingPointsProcessor::OutlierRemovingPointsProcessor(
    const double voxel_size, const double miss_per_hit_limit,
    PointsProcessor* next)
    : voxel_size_(voxel_size),
      miss_per_hit_limit_(miss_per_hit_limit),
      next_(next),
      state_(State::kPhase1),
      voxels_(voxel_size_) {
  LOG(INFO) << "Marking hits...";
}

void OutlierRemovingPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  switch (state_) {
    case State::kPhase1:
      ProcessInPhaseOne(*batch);
      break;

    case State::kPhase2:
      ProcessInPhaseTwo(*batch);
      break;

    case State::kPhase3:
      ProcessInPhaseThree(std::move(batch));
      break;
  }
}

PointsProcessor::FlushResult OutlierRemovingPointsProcessor::Flush() {
  switch (state_) {
    case State::kPhase1:
      LOG(INFO) << "Counting rays...";
      state_ = State::kPhase2;
      return FlushResult::kRestartStream;

    case State::kPhase2:
      LOG(INFO) << "Filtering outliers...";
      state_ = State::kPhase3;
      return FlushResult::kRestartStream;

    case State::kPhase3:
      CHECK(next_->Flush() == FlushResult::kFinished)
          << "Voxel filtering and outlier removal must be configured to occur "
             "after any stages that require multiple passes.";
      return FlushResult::kFinished;
  }
  LOG(FATAL) << "Invalid state.";
}

void OutlierRemovingPointsProcessor::ProcessInPhaseOne(
    const PointsBatch& batch) {
  for (size_t i = 0; i < batch.points.size(); ++i) {
    ++voxels_.mutable_value(voxels_.GetCellIndex(batch.points[i].position))
          ->hits;
  }
}

void OutlierRemovingPointsProcessor::ProcessInPhaseTwo(
    const PointsBatch& batch) {
  // TODO(whess): This samples every 'voxel_size' distance and could be improved
  // by better ray casting, and also by marking the hits of the current range
  // data to be excluded.
  std::vector<Eigen::Array3i> rays_indexs;
  rays_indexs.reserve(20);
  for (size_t i = 0; i < batch.points.size(); ++i) {
    const Eigen::Vector3f delta = batch.points[i].position - batch.origin;
    // weiji: judge laser angle type
    bool add_rays_at_end = true;
    Eigen::Vector3f norm = delta / delta.norm();
    if ((norm.z() < 0 && norm.z() > -0.5) || (norm.z() > 0.8)) {
      add_rays_at_end = false;
    }
    int miss_count = 0;
    rays_indexs.clear();
    // count rays by line
    BresenhamLine3D line(delta, voxel_size_);
    while (line.MoveNext()) {
      Eigen::Vector3f p = batch.origin + line.CurrentPos();
      const Eigen::Array3i index = voxels_.GetCellIndex(p);
      if (voxels_.value(index).hits > 0) {
        miss_count = 0;
        rays_indexs.push_back(index);
      } else {
        miss_count ++;
        // weiji: output rays when reaching an empty cell
        if (miss_count >= 2 && rays_indexs.size() > 0) {
          for (const auto& index : rays_indexs)
            ++voxels_.mutable_value(index)->rays;
          rays_indexs.clear();
        }
      }
    }
    if (add_rays_at_end && rays_indexs.size() > 0) {
      for (const auto& index : rays_indexs)
        ++voxels_.mutable_value(index)->rays;
    }
  }
}

void OutlierRemovingPointsProcessor::ProcessInPhaseThree(
    std::unique_ptr<PointsBatch> batch) {
  sensor::PointCloud points;
  std::vector<float> intensities;
  for (size_t i = 0; i < batch->points.size(); ++i) {
    const Eigen::Array3i index = voxels_.GetCellIndex(batch->points[i].position);
    const VoxelData& voxel = voxels_.value(index);
    if (!(voxel.rays < miss_per_hit_limit_ * voxel.hits)) {
    } else {
      if (voxel.grid_value == 0xff) {
        continue;
      }
      const GridDivision grid_quadrant = getGridDivision(index, batch->points[i].position);
      if (grid_quadrant != GridDivision::None) { 
        if (!voxel.check(grid_quadrant)) {
          voxels_.mutable_value(index)->set(grid_quadrant);
          points.push_back(batch->points[i]);
          intensities.push_back(batch->intensities[i]);
        }
      } else {
        LOG(ERROR) << "In Voxel filter, Grid Division is ERROR, Please check";
      } 
    }
  }
  batch->points = std::move(points);
  batch->intensities = std::move(intensities);
  next_->Process(std::move(batch));
}

OutlierRemovingPointsProcessor::GridDivision OutlierRemovingPointsProcessor::getGridDivision(
    const Eigen::Array3i& index, const Eigen::Vector3f& position) {
  bool x_sig = ((position[0] - index[0] * voxel_size_) >= 0);
  bool y_sig = ((position[1] - index[1] * voxel_size_) >= 0);
  bool z_sig = ((position[2] - index[2] * voxel_size_) >= 0);
  if (x_sig && y_sig && z_sig) {
    return GridDivision::Quadrant_0;
  } else if (!x_sig &&  y_sig &&  z_sig) {
     return GridDivision::Quadrant_1;
  } else if (!x_sig && !y_sig &&  z_sig) {
     return GridDivision::Quadrant_2;
  } else if ( x_sig && !y_sig &&  z_sig) {
     return GridDivision::Quadrant_3;
  } else if ( x_sig &&  y_sig && !z_sig) {
     return GridDivision::Quadrant_4;
  } else if (!x_sig &&  y_sig && !z_sig) {
     return GridDivision::Quadrant_5;
  } else if (!x_sig && !y_sig && !z_sig) {
     return GridDivision::Quadrant_6;
  } else if ( x_sig && !y_sig && !z_sig) {
     return GridDivision::Quadrant_7;
  }
  return GridDivision::None; 
}

}  // namespace io
}  // namespace cartographer
