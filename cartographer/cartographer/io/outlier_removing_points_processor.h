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

#ifndef CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/3d/hybrid_grid.h"

namespace cartographer {
namespace io {

// Voxel filters the data and only passes on points that we believe are on
// non-moving objects.
class OutlierRemovingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "voxel_filter_and_remove_moving_objects";

  OutlierRemovingPointsProcessor(double voxel_size, double miss_per_hit_limit,
                                 PointsProcessor* next);

  static std::unique_ptr<OutlierRemovingPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~OutlierRemovingPointsProcessor() override {}

  OutlierRemovingPointsProcessor(const OutlierRemovingPointsProcessor&) =
      delete;
  OutlierRemovingPointsProcessor& operator=(
      const OutlierRemovingPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  enum class State {
    kPhase1,
    kPhase2,
    kPhase3,
  };
  enum class GridDivision {
    Quadrant_0 = 0,
    Quadrant_1 = 1,
    Quadrant_2 = 2,
    Quadrant_3 = 3,
    Quadrant_4 = 4,
    Quadrant_5 = 5,
    Quadrant_6 = 6,
    Quadrant_7 = 7,
    None
  };

  // To reduce memory consumption by not having to keep all rays in memory, we
  // filter outliers in three phases each going over all data: First we compute
  // all voxels containing any hits, then we compute the rays passing through
  // each of these voxels, and finally we output all hits in voxels that are
  // considered obstructed.
  struct VoxelData {
    uint32_t rays;
    uint32_t hits : 24;
    uint32_t grid_value : 8;

    VoxelData () : rays(0), hits(0), grid_value(0) {}

    bool check(const GridDivision grid_quadrant) const {
      switch (grid_quadrant) {
        case GridDivision::Quadrant_0:
          return (grid_value & 0x01) > 0;
        case GridDivision::Quadrant_1:
          return (grid_value & 0x02) > 0;
        case GridDivision::Quadrant_2:
          return (grid_value & 0x04) > 0;
        case GridDivision::Quadrant_3:
          return (grid_value & 0x08) > 0;
        case GridDivision::Quadrant_4:
          return (grid_value & 0x10) > 0;
        case GridDivision::Quadrant_5:
          return (grid_value & 0x20) > 0;
        case GridDivision::Quadrant_6:
          return (grid_value & 0x40) > 0;
        case GridDivision::Quadrant_7:
          return (grid_value & 0x80) > 0;
        case GridDivision::None:
          return false;
      }
      return false;
    }

    void set(const GridDivision grid_quadrant) {
      switch (grid_quadrant) {
        case GridDivision::Quadrant_0:
          grid_value |= 0x01;
          break;
        case GridDivision::Quadrant_1:
          grid_value |= 0x02;
          break;
        case GridDivision::Quadrant_2:
          grid_value |= 0x04;
          break;
        case GridDivision::Quadrant_3:
          grid_value |= 0x08;
          break;
        case GridDivision::Quadrant_4:
          grid_value |= 0x10;
          break;
        case GridDivision::Quadrant_5:
          grid_value |= 0x20;
          break;
        case GridDivision::Quadrant_6:
          grid_value |= 0x40;
          break;
        case GridDivision::Quadrant_7:
          grid_value |= 0x80;
          break;
        case GridDivision::None:
          return;
      }
    }
  };

  // First phase counts the number of hits per voxel.
  void ProcessInPhaseOne(const PointsBatch& batch);

  // Second phase counts how many rays pass through each voxel. This is only
  // done for voxels that contain hits. This is to reduce memory consumption
  // by not adding data to free voxels.
  void ProcessInPhaseTwo(const PointsBatch& batch);

  // Third phase produces the output containing all inliers. We consider each
  // hit an inlier if it is inside a voxel that has a sufficiently high
  // hit-to-ray ratio.
  void ProcessInPhaseThree(std::unique_ptr<PointsBatch> batch);

  GridDivision getGridDivision(const Eigen::Array3i& index,
                                const Eigen::Vector3f& position);

  const double voxel_size_;
  const double miss_per_hit_limit_;
  PointsProcessor* const next_;
  State state_;
  mapping::HybridGridBase<VoxelData> voxels_;
};

// weiji: see http://members.chello.at/easyfilter/bresenham.html
class BresenhamLine3D
{
    Eigen::Vector3i start;
    Eigen::Vector3i current;
    Eigen::Vector3i end;
    int step_length;
    
    int dx, dy, dz;
    int sx, sy, sz;
    int dm, i;
    int x1, y1, z1;
    
    int max(int x, int y, int z)
    { return (x >= y && x >= z ? x : (y >= x && y >= z ? y : z)); }

public:
    BresenhamLine3D(Eigen::Vector3f end_pos_m, float step_m)
    {
        start = current = Eigen::Vector3i(0, 0, 0);
        end = Eigen::Vector3i((int)(end_pos_m[0] / 0.01f), (int)(end_pos_m[1] / 0.01f), (int)(end_pos_m[2] / 0.01f));
        step_length = abs((int)(step_m / 0.01f));
        
        dx = abs(end[0] - start[0]);
        dy = abs(end[1] - start[1]);
        dz = abs(end[2] - start[2]);
        sx = end[0] > start[0] ? step_length : -step_length;
        sy = end[1] > start[1] ? step_length : -step_length;
        sz = end[2] > start[2] ? step_length : -step_length;
        dm = max(dx, dy, dz);  /* maximum difference */
        i = dm / step_length;
        x1 = y1 = z1 = dm / 2; /* error offset */
    }
    BresenhamLine3D(Eigen::Vector3i start_pos, Eigen::Vector3i end_pos, int step)
        :start(start_pos), current(start_pos), end(end_pos), step_length(abs(step))
    {
        dx = abs(end[0] - start[0]);
        dy = abs(end[1] - start[1]);
        dz = abs(end[2] - start[2]);
        sx = end[0] > start[0] ? step_length : -step_length;
        sy = end[1] > start[1] ? step_length : -step_length;
        sz = end[2] > start[2] ? step_length : -step_length;
        dm = max(dx, dy, dz);  /* maximum difference */
        i = dm / step_length;
        x1 = y1 = z1 = dm / 2; /* error offset */
    }
    
    bool MoveNext()
    {
        if (i <= 0)
            return false;
        i--;
        x1 -= dx;
        if (x1 < 0) { x1 += dm; current[0] += sx; }
        y1 -= dy;
        if (y1 < 0) { y1 += dm; current[1] += sy; }
        z1 -= dz;
        if (z1 < 0) { z1 += dm; current[2] += sz; }
        return true;
    }
    
    Eigen::Vector3i CurrentValue()
    {
        return current;
    }
    
    Eigen::Vector3f CurrentPos()
    {
        Eigen::Vector3f pos(current[0] * 0.01f, current[1] * 0.01f, current[2] * 0.01f);
        return pos;
    }
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_
