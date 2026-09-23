#include "vio.h"
#include "preprocess.h"

#include <gtest/gtest.h>
#include <type_traits>

namespace
{
struct TrackedTree
{
  static int alive;
  std::unique_ptr<TrackedTree> child;
  int value;

  explicit TrackedTree(int value_in, bool with_child = false) : value(value_in)
  {
    ++alive;
    if (with_child) child = std::make_unique<TrackedTree>(value_in + 100);
  }
  ~TrackedTree() { --alive; }
};

int TrackedTree::alive = 0;

std::unique_ptr<Feature> makeFeature(VisualPoint *point, const cv::Mat &image, int patch_size = 64)
{
  auto feature = std::make_unique<Feature>(point, std::make_unique<float[]>(patch_size),
      V2D(64, 64), V3D(0, 0, 1), Sophus::SE3d(), 0);
  feature->img_ = image;
  feature->id_ = 0;
  feature->inv_expo_time_ = 1.0;
  return feature;
}

void addObservation(VisualPoint *point, const cv::Mat &image)
{
  auto feature = makeFeature(point, image);
  point->addFrameRef(feature.get());
  feature.release();
}

TEST(LruMap, CapacityOneDuplicateUpdateAndDeferredPrune)
{
  TrackedTree::alive = 0;
  LruMap<int, TrackedTree> map;
  map.configure(true, 1);
  static_assert(!std::is_copy_constructible<decltype(map)>::value);
  static_assert(!std::is_move_constructible<decltype(map)>::value);

  map.emplace(1, std::make_unique<TrackedTree>(1));
  EXPECT_EQ(map.size(), 1u);
  EXPECT_EQ(TrackedTree::alive, 1);

  auto replacement = map.insertOrAssign(1, std::make_unique<TrackedTree>(2));
  EXPECT_FALSE(replacement.second);
  EXPECT_EQ(map.size(), 1u);
  EXPECT_EQ(replacement.first->second->value, 2);
  EXPECT_EQ(TrackedTree::alive, 1);

  map.emplace(2, std::make_unique<TrackedTree>(3));
  EXPECT_EQ(map.size(), 2u);  // Borrowers remain valid until the explicit safe point.
  EXPECT_EQ(map.prune(), 1u);
  EXPECT_TRUE(map.contains(2));
  EXPECT_FALSE(map.contains(1));
  EXPECT_EQ(map.evictionCount(), 1u);
  EXPECT_TRUE(map.indexConsistent());
  map.clear();
  EXPECT_EQ(TrackedTree::alive, 0);
}

TEST(LruMap, TouchSelectsTailAndClearDestroysRecursiveValues)
{
  TrackedTree::alive = 0;
  LruMap<int, TrackedTree> map;
  map.configure(true, 2);
  map.emplace(1, std::make_unique<TrackedTree>(1, true));
  map.emplace(2, std::make_unique<TrackedTree>(2, true));
  EXPECT_EQ(TrackedTree::alive, 4);

  ASSERT_TRUE(map.touch(1));
  ASSERT_NE(map.mostRecentKey(), nullptr);
  ASSERT_NE(map.leastRecentKey(), nullptr);
  EXPECT_EQ(*map.mostRecentKey(), 1);
  EXPECT_EQ(*map.leastRecentKey(), 2);

  map.emplace(3, std::make_unique<TrackedTree>(3, true));
  EXPECT_EQ(map.prune(), 1u);
  EXPECT_TRUE(map.contains(1));
  EXPECT_TRUE(map.contains(3));
  EXPECT_FALSE(map.contains(2));
  EXPECT_EQ(TrackedTree::alive, 4);
  EXPECT_TRUE(map.indexConsistent());

  map.clear();
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(TrackedTree::alive, 0);
}

TEST(VoxelKey, FloorsNegativeCoordinatesAndSupportsNonDefaultSize)
{
  auto key = voxelLocation(V3D(-0.1, -1.25, -1.2501), 1.25);
  EXPECT_EQ(key.x, -1);
  EXPECT_EQ(key.y, -1);
  EXPECT_EQ(key.z, -2);
  key = voxelLocation(V3D(1.2499, 1.25, 2.5001), 1.25);
  EXPECT_EQ(key.x, 0);
  EXPECT_EQ(key.y, 1);
  EXPECT_EQ(key.z, 2);

  VoxelMapConfig config{};
  config.max_voxel_size_ = 1.25;
  VoxelMapManager manager(config);
  const V3D point(-0.1, 0.2, 2.6);
  const auto point_key = voxelLocation(point, config.max_voxel_size_);
  auto root = std::make_unique<VoxelOctoTree>(1, 0, 5, 50, 0.01f);
  auto *borrowed = root.get();
  manager.voxel_map_.emplace(point_key, std::move(root));
  EXPECT_EQ(manager.findVoxel(point), borrowed);
  EXPECT_EQ(static_cast<const VoxelMapManager &>(manager).findVoxel(point), borrowed);
  EXPECT_EQ(manager.findVoxel(V3D(1.3, 0.2, 2.6)), nullptr);
}
}

TEST(MemoryLifecycle, OwnersCannotBeCopiedAndEmptyManagersCanBeDestroyed)
{
  static_assert(!std::is_copy_constructible<VoxelMapManager>::value);
  static_assert(!std::is_copy_constructible<VoxelOctoTree>::value);
  static_assert(!std::is_copy_constructible<VIOManager>::value);
  static_assert(!std::is_copy_constructible<VOXEL_POINTS>::value);
  static_assert(!std::is_copy_constructible<Feature>::value);
  VoxelMapManager lidar;
  VIOManager visual;
  VoxelOctoTree empty_tree;
  EXPECT_EQ(lidar.collectMemoryStats().root_voxels, 0u);
  EXPECT_EQ(visual.collectMemoryStats().points, 0u);
}

TEST(MemoryLifecycle, PreprocessBlindRangeIsAlwaysInitialized)
{
  Preprocess preprocess;
  EXPECT_DOUBLE_EQ(preprocess.blind_sqr, preprocess.blind * preprocess.blind);
  preprocess.set(false, AVIA, 0.8, 1);
  EXPECT_DOUBLE_EQ(preprocess.blind_sqr, 0.64);
}

TEST(MemoryLifecycle, LidarSnapshotTracksChildrenAndSlidingDeletion)
{
  VoxelMapManager manager;
  auto root = std::make_unique<VoxelOctoTree>(2, 0, 5, 50, 0.01f);
  root->leaves_[3] = new VoxelOctoTree(2, 1, 5, 50, 0.01f);
  root->temp_points_.resize(2);
  root->temp_points_.reserve(12);
  root->leaves_[3]->temp_points_.resize(3);
  const auto retained_capacity = root->temp_points_.capacity() + root->leaves_[3]->temp_points_.capacity();
  manager.voxel_map_.emplace(VOXEL_LOCATION(0, 0, 0), std::move(root));
  manager.voxel_map_.emplace(VOXEL_LOCATION(10, 0, 0), std::make_unique<VoxelOctoTree>(2, 0, 5, 50, 0.01f));

  auto stats = manager.collectMemoryStats();
  EXPECT_EQ(stats.root_voxels, 2u);
  EXPECT_EQ(stats.octree_nodes, 3u);
  EXPECT_EQ(stats.planes, 3u);
  EXPECT_EQ(stats.buffered_points, 5u);
  EXPECT_EQ(stats.buffered_point_capacity, retained_capacity);

  manager.clearMemOutOfMap(1, -1, 1, -1, 1, -1);
  stats = manager.collectMemoryStats();
  EXPECT_EQ(stats.root_voxels, 1u);
  EXPECT_EQ(stats.octree_nodes, 2u);
  EXPECT_EQ(stats.buffered_points, 5u);
  // The remaining root and its child must also be freed on manager destruction.
}

TEST(MemoryLifecycle, SlidingAndLruDeletionKeepBothIndexesConsistent)
{
  VoxelMapManager manager;
  manager.configureLru(true, 2);
  const auto add_root = [&manager](int x)
  {
    manager.voxel_map_.emplace(VOXEL_LOCATION(x, 0, 0), std::make_unique<VoxelOctoTree>(1, 0, 5, 50, 0.01f));
  };

  add_root(0);
  add_root(10);
  add_root(20);
  manager.clearMemOutOfMap(15, -1, 1, -1, 1, -1); // Spatial deletion removes 20.
  EXPECT_EQ(manager.voxel_map_.size(), 2u);
  EXPECT_TRUE(manager.voxel_map_.indexConsistent());

  add_root(30);
  ASSERT_TRUE(manager.voxel_map_.touch(VOXEL_LOCATION(0, 0, 0)));
  EXPECT_EQ(manager.pruneLru(), 1u); // Recency deletion removes 10.
  EXPECT_TRUE(manager.voxel_map_.contains(VOXEL_LOCATION(0, 0, 0)));
  EXPECT_TRUE(manager.voxel_map_.contains(VOXEL_LOCATION(30, 0, 0)));
  EXPECT_FALSE(manager.voxel_map_.contains(VOXEL_LOCATION(10, 0, 0)));
  EXPECT_TRUE(manager.voxel_map_.indexConsistent());

  manager.clearMemOutOfMap(35, 25, 1, -1, 1, -1); // Spatial deletion removes 0.
  EXPECT_EQ(manager.voxel_map_.size(), 1u);
  EXPECT_TRUE(manager.voxel_map_.contains(VOXEL_LOCATION(30, 0, 0)));
  EXPECT_TRUE(manager.voxel_map_.indexConsistent());
}

TEST(MemoryLifecycle, VisualSnapshotDeduplicatesSharedImagesAndRois)
{
  VIOManager manager;
  manager.patch_size_total = 64;
  cv::Mat image(128, 128, CV_8UC1, cv::Scalar(100));
  cv::Mat independent = image.clone();
  auto point = std::make_unique<VisualPoint>(V3D(0, 0, 3));
  auto *borrowed = point.get();
  addObservation(borrowed, image);
  addObservation(borrowed, image(cv::Rect(16, 16, 32, 32)));
  addObservation(borrowed, independent);
  manager.insertPointIntoVoxelMap(std::move(point));
  manager.insertPointIntoVoxelMap(std::make_unique<VisualPoint>(V3D(0.1, 0.1, 3.1)));

  auto stats = manager.collectMemoryStats();
  EXPECT_EQ(stats.voxels, 1u);
  EXPECT_EQ(stats.points, 2u);
  EXPECT_EQ(stats.observations, 3u);
  EXPECT_EQ(stats.patches, 3u);
  EXPECT_EQ(stats.patch_bytes, 3u * 64 * sizeof(float));
  EXPECT_EQ(stats.image_allocations, 2u);
  EXPECT_EQ(stats.image_bytes, image.u->size + independent.u->size);

  borrowed->deleteFeatureRef(borrowed->obs_.front());
  stats = manager.collectMemoryStats();
  EXPECT_EQ(stats.observations, 2u);
  EXPECT_EQ(stats.image_allocations, 1u);
  EXPECT_EQ(stats.image_bytes, image.u->size);

  manager.feat_map.clear();
  stats = manager.collectMemoryStats();
  EXPECT_EQ(stats.points, 0u);
  EXPECT_EQ(stats.patches, 0u);
  EXPECT_EQ(stats.image_bytes, 0u);
}

TEST(MemoryLifecycle, VisualFrameKeepsActivePointersUntilSafePrune)
{
  VIOManager manager;
  manager.patch_size_total = 64;
  manager.configureLru(true, 1);
  manager.visual_submap = std::make_unique<SubSparseMap>();
  manager.retrieve_voxel_points.resize(1, nullptr);

  cv::Mat first_image(64, 64, CV_8UC1, cv::Scalar(10));
  cv::Mat second_image(64, 64, CV_8UC1, cv::Scalar(20));
  auto active = std::make_unique<VisualPoint>(V3D(0.1, 0.1, 0.1));
  auto *active_borrowed = active.get();
  addObservation(active_borrowed, first_image);
  manager.insertPointIntoVoxelMap(std::move(active));
  manager.visual_submap->voxel_points.push_back(active_borrowed);
  manager.retrieve_voxel_points[0] = active_borrowed;
  ASSERT_TRUE(manager.feat_map.touch(voxelLocation(active_borrowed->pos_, manager.visual_voxel_size)));

  auto added = std::make_unique<VisualPoint>(V3D(1.1, 0.1, 0.1));
  addObservation(added.get(), second_image);
  manager.insertPointIntoVoxelMap(std::move(added));
  EXPECT_EQ(manager.feat_map.size(), 2u);
  EXPECT_DOUBLE_EQ(active_borrowed->pos_.x(), 0.1); // Still valid before frame-end pruning.
  EXPECT_EQ(manager.collectMemoryStats().image_allocations, 2u);

  EXPECT_EQ(manager.finishFrameAndPrune(), 1u);
  EXPECT_EQ(manager.feat_map.size(), 1u);
  EXPECT_TRUE(manager.visual_submap->voxel_points.empty());
  EXPECT_EQ(manager.retrieve_voxel_points[0], nullptr);
  EXPECT_EQ(manager.collectMemoryStats().image_allocations, 1u);
  EXPECT_TRUE(manager.feat_map.indexConsistent());
}

TEST(MemoryLifecycle, NonNormalRetrievalClearsOwnedWarpCache)
{
  VIOManager manager;
  manager.normal_en = false;
  manager.raycast_en = false;
  manager.width = manager.height = 32;
  manager.length = 0;
  manager.visual_submap = std::make_unique<SubSparseMap>();
  manager.insertPointIntoVoxelMap(std::make_unique<VisualPoint>(V3D(0, 0, 3)));
  cv::Mat image(32, 32, CV_8UC1, cv::Scalar(0));
  std::vector<pointWithVar> points;
  VoxelMap planes;
  for (int frame = 0; frame < 8; ++frame)
  {
    for (int id = 0; id < 128; ++id)
      manager.warp_map.emplace(id, std::make_unique<Warp>(0, Matrix2d::Identity()));
    EXPECT_EQ(manager.collectMemoryStats().warp_entries, 128u);
    manager.retrieveFromVisualSparseMap(image, points, planes);
    EXPECT_TRUE(manager.warp_map.empty());
  }
  // Also exercise destruction with a nonempty cache (checked under Memcheck).
  manager.warp_map.emplace(0, std::make_unique<Warp>(0, Matrix2d::Identity()));
}

TEST(MemoryLifecycle, VisualUpdateRetainsOnlyAcceptedPatches)
{
  // Keep camera-map initialization from leaving TBB worker TLS alive at exit.
  // This affects only this ownership test, not the mapping executable.
  cv::setNumThreads(1);
  vk::PinholeCamera camera(128, 128, 1.0, 50, 50, 64, 64);
  cv::Mat image(128, 128, CV_8UC1, cv::Scalar(100));
  StatesGroup state;
  state.inv_expo_time = 1.0;
  VIOManager manager;
  manager.cam = &camera;
  manager.state = &state;
  manager.width = manager.height = 128;
  manager.patch_size = 8;
  manager.patch_size_half = 4;
  manager.patch_size_total = 64;
  manager.total_points = 1;
  manager.update_flag.assign(1, 0);
  manager.new_frame_ = std::make_unique<Frame>(&camera, image);
  manager.new_frame_->T_f_w_ = Sophus::SE3d();
  manager.visual_submap = std::make_unique<SubSparseMap>();
  manager.visual_submap->search_levels.push_back(0);

  auto point = std::make_unique<VisualPoint>(V3D(0, 0, 3));
  auto *borrowed = point.get();
  addObservation(borrowed, image);
  manager.visual_submap->voxel_points.push_back(borrowed);
  manager.insertPointIntoVoxelMap(std::move(point));

  // No motion: repeatedly take the formerly leaking add_flag=false branch.
  for (int frame = 0; frame < 32; ++frame) manager.updateVisualMapPoints(image);
  EXPECT_EQ(borrowed->obs_.size(), 1u);
  EXPECT_EQ(manager.collectMemoryStats().patch_bytes, 64u * sizeof(float));

  manager.new_frame_->T_f_w_ = Sophus::SE3d(Sophus::SO3d(), V3D(0.6, 0, 0));
  manager.updateVisualMapPoints(image);
  ASSERT_EQ(borrowed->obs_.size(), 2u);
  EXPECT_EQ(manager.update_flag[0], 1);
  EXPECT_EQ(manager.collectMemoryStats().patch_bytes, 2u * 64 * sizeof(float));
  for (int i = 0; i < 64; ++i) EXPECT_FLOAT_EQ(borrowed->obs_.front()->patch_[i], 100.0f);

  borrowed->ref_patch = borrowed->obs_.front();
  borrowed->has_ref_patch_ = true;
  borrowed->is_converged_ = true;
  manager.updateVisualMapPoints(image);
  EXPECT_EQ(borrowed->obs_.size(), 1u);
  EXPECT_EQ(manager.collectMemoryStats().patch_bytes, 64u * sizeof(float));
}
