#ifndef FAST_LIVO_MEMORY_STATS_H_
#define FAST_LIVO_MEMORY_STATS_H_

#include <cstddef>

// Snapshot counts, not an estimate of the complete process heap or RSS.
struct LidarMapMemoryStats
{
  std::size_t root_voxels = 0;
  std::size_t octree_nodes = 0;
  std::size_t planes = 0;
  std::size_t buffered_points = 0;
  std::size_t buffered_point_capacity = 0;
};

struct VisualMapMemoryStats
{
  std::size_t voxels = 0;
  std::size_t points = 0;
  std::size_t observations = 0;
  std::size_t patches = 0;
  std::size_t patch_bytes = 0;
  // Count shared cv::Mat allocations only once, including ROI aliases.
  std::size_t image_allocations = 0;
  std::size_t image_bytes = 0;
  std::size_t warp_entries = 0;
};

#endif
