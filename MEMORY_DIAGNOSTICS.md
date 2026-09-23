# 内存释放与统计

本次修改修复 patch/Warp 的释放路径，明确两张地图和相机对象的所有权，并为激光、视觉地图接入共享 LRU。ROS2 launch 把 mapping/camera YAML 直接传给建图节点，正常启动默认不使用 Valgrind；`mapping_prefix` 可用于短序列检查。匹配阈值、视觉/激光算法流程、订阅 QoS 和滑窗默认开关保持原配置。

## 释放路径

- `Feature::patch_` 使用 `unique_ptr<float[]>`。`updateVisualMapPoints()` 只有决定保留新观测后才分配 patch；生成视觉点和追加观测时，也用临时智能指针保护尚未交给地图/观测列表的对象。
- `warp_map` 使用 `unique_ptr<Warp>`，原有 `clear()` 会释放缓存内容；清理时机保持原样。
- 激光根地图 `VoxelMap` 和视觉根地图 `feat_map` 持有 `unique_ptr`。滑窗 erase、clear、管理器析构都会释放对象。八叉树的现有递归析构和视觉点的现有观测析构继续负责释放下层对象。
- 删除 `LIVMapper` 的冗余裸指针地图和管理器的浅拷贝构造。拥有对象的管理器、八叉树和视觉体素禁止复制。
- `visual_submap` 使用智能指针；八叉树空构造的 plane/children 指针及视觉点的 `ref_patch` 初始化为空，保证未完成初始化时也能安全销毁。
- 相机加载器返回的裸指针由 `VIOManager::owned_cam` 接管；测试仍可注入不接管的借用相机。

## LRU 参数与安全点

`config/mid360.yaml` 的默认值是：

```yaml
lru:
  enabled: true
  lidar_capacity: 30000
  visual_capacity: 2000
```

参数为启动后只读；启用 LRU 时两套容量必须大于 0。哈希表负责 O(1) 平均定位，链表表头为 MRU、表尾为 LRU。插入本身不隐式淘汰：激光在一次初建/更新批次结束后裁剪；视觉在绘制、观测更新、参考 patch 和可选 Colmap 消费完成，并清空 `visual_submap`、`retrieve_voxel_points` 等借用指针后裁剪。因此容量约束是视觉帧末上限，帧内可以临时超过容量。

所有激光/视觉 key 都通过 `floor(coord / voxel_size)` 生成。视觉使用 0.5 m；激光查询使用 `lio.voxel_size` 的真实配置值。滑窗和 LRU 均调用同一容器的 `erase()`，保证哈希索引和链表同步。

## 默认轻量统计

默认每 5 秒输出 `[Memory]`，使用单调时钟，不依赖 ROS 仿真时间。位于主循环的同步之前，同步等待期间也会输出；在当前串行主循环内访问地图，本地输入队列按原有互斥锁读取。

| 字段 | 含义 |
| --- | --- |
| `rss_kib` | `/proc/self/status` 中进程驻留内存，KiB；不可用时为 −1 |
| `lidar_roots` | 激光根体素条目数，不包含子八叉树 |
| `visual_voxels` | 视觉体素条目数 |
| `warp_entries` | 当前 Warp 缓存条目数 |
| `lru[...]` | LRU 开关、两套容量及激光/视觉累计淘汰数 |
| `local_queues[...]` | 本地 LiDAR、IMU、图像、传播 IMU deque 长度；不包含 DDS 内部队列 |
| `path_poses` | 内存中轨迹的 pose 数 |
| `pcd_rgb_points` / `pcd_intensity_points` | 两个保存缓存当前的点数 |
| `pending_color_points` | 等待颜色投影/发布的累计点数 |

原 VIO 的 `Sparse Map Size` 被参数同名遮蔽，实际读取激光地图；现在改为 `Visual Map Voxels`，读取视觉成员地图。

## 可选详细统计

在启动所用 mapping YAML 的 `/** → ros__parameters` 下增加以下块，即可同时输出 `[MemoryDetail]`：

```yaml
memory_stats:
  enabled: true
  detailed: true
  interval_sec: 5.0
```

不添加时，默认 `enabled=true`、`detailed=false`、`interval_sec=5.0`。设置 `enabled=false` 可关闭统计。参数启动后只读，间隔必须为有限正数。可以使用独立的 MID360 参数文件，并通过现有 `mapping_params_file` launch 参数传入，避免修改原文件。

详细统计会遍历当前地图，包含：

- 激光八叉树总节点、已分配 plane 对象、缓存点数量及点向量 capacity 总和。`planes` 包含尚未拟合为有效平面的对象；`buffered_points` 不代表历史累计输入点。
- 视觉点、观测、持有的 patch 数量和 patch 数组字节数。
- 视觉观测引用的图像分配数量和字节数。共享同一幅图像或其 ROI 时按底层存储去重；OpenCV 管理的图像计入完整分配大小，外部存储只能计入 Mat 可见的底层跨度。
- `snapshot_ms` 显示本次采样耗时，包括轻量采样和详细遍历。

这些是当前可达对象的快照，不是全进程堆大小。图像字节数只覆盖视觉观测引用的存储，不包含所有输入队列、当前帧副本、容器开销及分配器开销；不能和 RSS 相加。详细遍历随地图增长而变慢，所以默认关闭，采样结束后再安排下次采样。

## 回归验证

`test/test_memory_lifecycle.cpp` 当前有 11 个用例，覆盖容量 1、重复更新、touch 后尾部淘汰、递归析构、clear、滑窗/LRU 交替删除、活跃视觉体素与新体素同时超容量、负坐标、1.25 m 激光查询，以及原有 patch/Warp、共享图像统计和预处理初始化路径。

可以用 `colcon test --packages-select fast_livo` 运行，并对 `build/fast_livo/test_memory_lifecycle` 使用 Valgrind Memcheck 检查退出后的泄漏和非法访问。这些定向测试不替代长 rosbag 的内存趋势及轨迹精度验证。

2026-09-16 验证结果：ROS2 Release 主程序和测试目标编译通过；CTest 中 11 个 GTest 全部通过。定向 Memcheck 报告 0 errors，definitely/indirectly/possibly lost 均为 0，未使用 suppression。仍有依赖库的可达分配，因此不宣称退出时堆完全为空。

首次 Memcheck 在 OpenCV 默认并行下报告 3456 字节的 TBB 工作线程 TLS 为 possibly lost，调用栈来自 OpenCV/TBB/pthread。最终测试在相机初始化前使用 `cv::setNumThreads(1)`，排除后台线程退出时的 TLS 干扰；仅测试采用这个设置，生产程序的 OpenCV/OpenMP 并发配置没有改变。

同日对 352.4217 秒、40.1 GiB 的 `FAST_LIVO2_newpc` 做了两次 1× 完整回放，均使用正常 Release 启动且不挂内存检查器：

| 模式 | 结果 | 地图与内存终值 |
| --- | --- | --- |
| 纯 LIO (`common.img_en: 0`) | `/cloud_registered` 约 10 Hz，3513 个 path pose，完整跑到 bag 结束 | 激光 30000、视觉 0；激光淘汰 4221；RSS 约 366 MiB |
| LIVO | LIO/VIO 持续更新，7019 个 path pose，完整跑到 bag 结束 | 激光/视觉 30000/2000；淘汰 8911/10290；RSS 在约 3.17 GiB 形成平台并保持到结束 |

两次回放均无 `process has died`、崩溃、`bad_alloc` 或 rosbag `Message queue starved`。LIVO 尾段曾短暂积压 16 个 LiDAR、52 个图像，bag 结束后 LiDAR 队列回到 0，并出现同步追赶警告；这属于性能/同步现象，不是 LRU 生命周期错误。

短 LIVO 序列也通过 launch 的 `mapping_prefix` 在 Valgrind 下运行，实际覆盖了 LIO 建图和视觉新增/检索/更新。没有发现 LRU/八叉树/VisualPoint/Feature 的非法读写。完整 ROS2 进程的报告包含 FastDDS、pluginlib、OpenCV/TBB 线程以及 SIGTERM 介入依赖析构产生的退出噪声；所以泄漏门禁采用可确定退出的 11 个定向测试，真实回放用于确认访问安全和运行行为。
