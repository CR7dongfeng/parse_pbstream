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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_H_

#include <array>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/optimization/cost_functions/cost_helpers.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace cartographer {
namespace mapping {
namespace optimization {

class SpaCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const PoseGraph::Constraint::Pose& pose) {
    return new ceres::AutoDiffCostFunction<
        SpaCostFunction3D, 6 /* residuals */, 4 /* rotation variables */,
        3 /* translation variables */, 4 /* rotation variables */,
        3 /* translation variables */>(new SpaCostFunction3D(pose));
  }
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const PoseGraph::Constraint::Pose& pose, bool unfixed_z) {
    return new ceres::AutoDiffCostFunction<
        SpaCostFunction3D, 6 /* residuals */, 4 /* rotation variables */,
        3 /* translation variables */, 4 /* rotation variables */,
        3 /* translation variables */>(new SpaCostFunction3D(pose, unfixed_z));
  }

  template <typename T>
  bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                  const T* const c_j_rotation, const T* const c_j_translation,
                  T* const e) const {
    std::array<T, 6> error;
    if (!unfixed_z_) {
      error = ScaleError(
          ComputeUnscaledError(pose_.zbar_ij, c_i_rotation, c_i_translation,
                              c_j_rotation, c_j_translation),
          pose_.translation_weight, pose_.rotation_weight);
    } else {
      error = ScaleErrorUnfixedZ(
          ComputeUnscaledError(pose_.zbar_ij, c_i_rotation, c_i_translation,
                              c_j_rotation, c_j_translation),
          pose_.translation_weight, pose_.rotation_weight);
    }
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  explicit SpaCostFunction3D(const PoseGraph::Constraint::Pose& pose)
      : pose_(pose), unfixed_z_(false) {}
  explicit SpaCostFunction3D(const PoseGraph::Constraint::Pose& pose, bool unfixed_z)
      : pose_(pose), unfixed_z_(unfixed_z) {}

  const PoseGraph::Constraint::Pose pose_;
  bool unfixed_z_;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_H_
