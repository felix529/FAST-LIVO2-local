# FAST-LIVO2 LRU 遍历与 ROS2 实现

2026-09-16。本文记录参考仓库的源码审查、本地 ROS2 实现及回放验证。patch/Warp 释放、根地图所有权、共享 LRU、统一体素 key、边界测试和 LIO/LIVO 回放均已完成，详见 [MEMORY_DIAGNOSTICS.md](MEMORY_DIAGNOSTICS.md)。参考仓库的行号仍固定在审查时的提交。

参考仓库检查的是 `yqmy0814/FAST-LIVO2` 的 `devel` 分支，提交 `48a2a32b8efd2267c36e01009fbe89312598ece9`。本地检查的是 `src/FAST-LIVO2`，HEAD 为 `33922c29ce45f2913766b4aa9907f0c6d714b21b`，包含工作区已有的 CMake、MID360 配置及 launch 修改。

**结论：可以采用 PPT 的“哈希索引 + 双向链表”，但需要同时覆盖激光地图和视觉地图；移植的主要难点是对象所有权和淘汰时机。ROS2 参数、并发和消息缓存也必须按本地实现处理。**

## 参考仓库的完整有效调用链

源码里保留了一些 `#if 0` 的旧函数。以下列出实际调用的 LRU 路径，避免把旧版本和当前实现混在一起。

| 环节 | 参考源码位置 | 实际行为 |
| --- | --- | --- |
| 参数入口 | `src/LIVMapper.cpp:47` | ROS1 参数 `lru_config/lru_size`，默认 1,000,000；同一个数分别赋给激光和视觉管理器 |
| 激光容器 | `include/voxel_map.h:188` | `VMData = pair<VOXEL_LOCATION, VoxelOctoTree*>`；`vm_data_` 保存节点，`vm_map_` 保存链表迭代器 |
| 初始建图 | `src/LIVMapper.cpp:386` → `src/voxel_map.cpp:626` | `BuildVoxelMapLRU()`：命中后更新并移到表头，新节点插表头；超限删除表尾；最后初始化保留的八叉树 |
| 激光增量更新 | `src/LIVMapper.cpp:449` → `src/voxel_map.cpp:759` | `UpdateVoxelMapLRU()`：更新后 `splice`，插入后检查容量并删除旧节点 |
| 激光匹配 | `StateEstimation()` → `src/voxel_map.cpp:898` | `BuildResidualListLRU()` 通过 `iter->second->second` 取八叉树；包括邻体素查询；没有刷新链表 |
| 平面显示 | `src/voxel_map.cpp:1085` | `PubVoxelMapLRU()` 适配新的索引类型；遍历不刷新链表 |
| 视觉容器 | `include/vio.h:81,131` | `VPData = pair<VOXEL_LOCATION, VOXEL_POINTS*>`；独立的 `vp_data_` / `vp_map_` |
| 视觉插入 | `GenerateVisualMapPoints()` → `src/vio.cpp:257` | `InsertPointIntoFeatureMapLRU()` 在已有体素加入点并移到表头；新体素插表头，随后立即淘汰 |
| 视觉检索 | `ProcessFrame()` → `src/vio.cpp:776` | `RetrieveFromVisualSparseMapLRU()` 查询视觉体素，raycast 分支也读取激光地图 |
| 视觉触摸 | `src/vio.cpp:1133` | 点通过图像误差/NCC 筛选、加入当前子地图后，将所属视觉体素移到表头 |
| 参考 patch 更新 | `src/vio.cpp:1467` | 接受激光 LRU 索引的重载，从激光八叉树读取平面、更新视觉点；没有刷新激光 LRU |
| 释放 | `src/vio.cpp:21` 和两类体素析构函数 | 视觉管理器退出时删除剩余体素；淘汰时递归释放点及观测。激光管理器未提供清理剩余根指针的析构函数 |
| 原滑窗 | `src/voxel_map.cpp:1229,1262` | `MapSliding()` / `ClearMemOutOfMap()` 函数体已注释，虽然上层仍保留调用 |

对应固定版本：[激光实现](https://github.com/yqmy0814/FAST-LIVO2/blob/48a2a32b8efd2267c36e01009fbe89312598ece9/src/voxel_map.cpp)、[视觉实现](https://github.com/yqmy0814/FAST-LIVO2/blob/48a2a32b8efd2267c36e01009fbe89312598ece9/src/vio.cpp)、[参数入口](https://github.com/yqmy0814/FAST-LIVO2/blob/48a2a32b8efd2267c36e01009fbe89312598ece9/src/LIVMapper.cpp)。

因此，参考实现的“最近使用”有两个不同定义：激光侧是最近插入/更新；视觉侧还包括最近被成功选入当前帧的点。不是所有 `find()` 都会刷新 LRU。

`std::list::splice()` 只改变节点链接，已有迭代器仍指向原节点，因此紧随其后的 `iter->second = data.begin()` 不是必需的。平均 O(1) 指的是哈希定位、移动一个链表节点和移除索引；递归析构八叉树或大量视觉点的开销仍随对象数量增加。

## 不能直接照搬的细节

**容量边界。** 两种地图均在插入后使用 `size() >= lru_size_`，正常正容量 C 实际最多保留 C−1 个体素。激光增量更新在淘汰之后还会通过 `vm_map_[position]` 初始化新体素；C=1 时刚插入的节点已经删除，后续解引用无效。应在启用时验证容量 > 0，并在安全位置使用 `while (size() > capacity)`。关闭功能用独立开关表达，不把负数转为 `size_t`，也不让 0 的语义含糊。

**本帧引用保护。** 参考视觉流程是“检索旧点 → EKF → 添加新点 → 绘制旧点 → 更新旧点/参考 patch”。旧点以裸指针保存在 `visual_submap`。添加新体素时立即淘汰，在容量不足以同时容纳活跃体素和新体素时，会删掉后续仍需使用的点。刷新到表头降低了概率，但不构成生命周期保护。这是源码可推导的触发条件，尚未用真实数据复现崩溃。

**负坐标索引。** 本地与参考都有不一致的离散化：插入使用“除体素尺寸，负数再减 1，最后截断”；视觉子地图检索却使用 `floor()` 后负数再减 1。例如尺寸 0.5、x=−0.1，前者为 −1，后者为 −2；x=−0.5 时二者又都偏离正确的 −1。建议统一到 `int64_t(std::floor(x / voxel_size))`，覆盖建图、更新、残差、视觉检索、raycast、参考 patch 和滑窗。该修改会影响负坐标匹配，应单独验证，而不能只替换一个入口。

**两套尺寸。** 视觉体素目前固定 0.5 m，激光体素来自配置。本地 MID360 恰好也是 0.5 m，但 VIO 内查询激光平面的地方仍硬编码 0.5。应让每个地图自己根据其尺寸生成 key，避免以后调激光尺寸时查错地图。不能假设两种地图的同一 key 永远对应同一空间块，也不应仅凭 key 相同就联动删除。

## 本地工程的实现落点

| 本地位置 | 实现 |
| --- | --- |
| `include/lru_map.h` | 共享的 owning hash + MRU/LRU list；禁止复制/移动，显式 `prune()`，统一 `erase/clear` 和淘汰计数 |
| `include/voxel_map.h` | 激光根地图改为 `LruMap<VOXEL_LOCATION, VoxelOctoTree>`；根对象由 `unique_ptr` 唯一持有 |
| `src/voxel_map.cpp` | 初建、增量更新命中时 touch，批次末裁剪；残差并行阶段只读；发布和滑窗通过同一容器遍历/删除 |
| `include/vio.h`、`src/vio.cpp` | 视觉地图使用同一 LRU；插入和成功使用时 touch；清空帧内借用指针后才在帧末裁剪 |
| `include/voxel_map.h`、激光/视觉调用点 | key 统一为 `floor(coord / voxel_size)`；视觉尺寸和激光配置尺寸分别传递 |
| `src/LIVMapper.cpp`、`config/mid360.yaml` | ROS2 只读参数 `lru.enabled/lidar_capacity/visual_capacity`，启用时容量必须大于 0 |
| `test/test_memory_lifecycle.cpp` | 11 个用例覆盖边界、删除交替、递归析构、帧内指针保护和非 0.5 m 查询 |

上述位置以审查时工作区为准。参考仓库还混有函数重命名、点类型、Sophus、glog/gflags、ROS1 启动方式和模式删除等改动，不需要为了 LRU 一并移植。

## 建议的数据结构和处理顺序

实现抽出了不依赖 ROS 的 `LruMap<Key, T>`，由两类地图共用。核心结构如下：

```cpp
template<class T>
class LruVoxelMap {
  using Entry = std::pair<VOXEL_LOCATION, std::unique_ptr<T>>;
  using Entries = std::list<Entry>;

  Entries entries_;  // 表头 MRU，表尾 LRU；唯一拥有根体素
  std::unordered_map<VOXEL_LOCATION, typename Entries::iterator> index_;
  // find()/peek()：不修改顺序，返回临时借用指针
  // touch()：splice 到表头
  // emplace()：创建、建立索引；不隐式触发不安全的淘汰
  // erase()/prune()/clear()：统一维护两个容器
};
```

实际实现遵守上述结构，并禁止复制和移动，避免索引迭代器引用错误的链表。插入处理重复 key，哈希建索引失败时回滚链表节点；删除先维护哈希索引，再由 `unique_ptr` 执行递归析构。

激光释放链是“根体素 → 子八叉树 → 各自的 plane 和点缓存”；视觉释放链是“体素 → VisualPoint → Feature → patch 数组，以及 cv::Mat 引用计数”。现有下层析构函数基本已具备这条链。`unique_ptr` 保证释放所有权，但不能保护仍被其他地方借用的裸指针。

激光侧保持参考的“插入/更新触摸”语义，匹配查询不改顺序并保留本地 OpenMP。建图/增量更新完成后串行裁剪，不在并行残差循环中修改链表。

视觉侧已按以下顺序实现：

```text
清空上一帧临时引用（应在“地图为空”的提前返回之前完成）
→ 检索并 touch 成功使用的体素
→ EKF 更新
→ 添加视觉点（只创建/更新/touch，暂不淘汰）
→ 绘制、更新视觉点、更新参考 patch、Colmap 输出
→ 清空 visual_submap / retrieve_voxel_points 等临时裸指针
→ prune 到 visual_capacity
```

本地 `LIVMapper` 在 `processFrame()` 后使用的是图像和 frame 的投影接口，没有继续消费这两组视觉点裸指针。这样可以在 VIO 帧末安全回收。其容量承诺是**帧末不超过 C**；帧内可能暂时多出本帧新增的体素。如果必须逐次插入都保持硬上限，则需要 pin 当前帧引用，并在没有可淘汰节点时拒绝/延迟新点，不能一边保留裸指针一边强行删除。

滑窗与 LRU 可以同时存在：滑窗负责空间范围，LRU 负责数量。两者都经过容器 `erase`，同时维护哈希和链表索引。默认仍沿用 `map_sliding_en: false`。

## ROS2 接入方式

当前工程已经使用 `rclcpp`、ament、C++17、ROS2 消息和 Python launch。数据结构本身不需要 ROS1 依赖；沿用本地消息类型、Sophus 接口和构建系统。

参数分为 `lru.enabled`、`lru.lidar_capacity`、`lru.visual_capacity`。MID360 当前默认值已用 352.4 秒 rosbag 验证：

```yaml
/**:
  ros__parameters:
    lru:
      enabled: true
      lidar_capacity: 30000
      visual_capacity: 2000
```

在 mapping 节点中使用 `declare_parameter<bool>("lru.enabled", ...)` 和 `declare_parameter<int64_t>("lru.lidar_capacity", ...)` 等接口，先校验再转内部容量类型。启动文件已有 `parameters=[mapping_params_file]`，不需要另建 ROS1 参数服务器，也无需把这些参数放入相机的 `parameter_blackboard`。首版可通过参数描述符设为启动后只读；如果以后支持动态缩容，参数回调只提交请求，由建图线程在安全位置执行裁剪。

本地 `run()` 在 `spin_some()` 后执行建图，当前主流程是串行的；不能仅因采用 ROS2 就认定地图已多线程并发。实际已有的并发点是激光残差和视觉优化的 OpenMP，后者同样应保持帧内对象存活。以后切到多线程 executor 或独立处理线程时，必须重新约束地图访问与删除，不能只给哈希查找加一个短锁就认为返回的裸指针一直有效。

参考 ROS1 launch 还设置了 `MALLOC_ARENA_MAX=4` 和 `MALLOC_TRIM_THRESHOLD_=1073741824`。这是分配器调优，不属于 LRU。ROS2 如果确实需要，可通过节点 `additional_env` 注入；不建议第一版照搬数值或每帧主动 trim。本地 CMake 也会可选链接 mimalloc，需要先确认实际分配器，不能把 glibc 参数当作通用内存上限。

## LRU 以外必须纳入内存排查的项目

1. **patch 释放路径（已修复）**：仅在观测确定保留后分配 patch，并使用 `unique_ptr<float[]>` 转交所有权。
2. **Warp 释放路径（已修复）**：`warp_map` 持有 `unique_ptr<Warp>`，`clear()` 会释放缓存。
3. **视觉体素内点数未设上限**：体素数量受限不等于每个体素内点数受限。单点观测已有约 30 个的限制，但每个 `VOXEL_POINTS::voxel_points` 仍可继续增加。需要另外制定每体素点数限制或去重/质量替换策略；若删除已有点，也必须遵守帧末引用保护。
4. **历史图像保留**：`Feature::img_ = img` 共享整幅图像的存储，并非只存 8×8 patch。按相机配置，1280×960 的 8 位灰度像素约 1.17 MiB/幅；同帧多个 Feature 共享像素，不能按 Feature 数重复相乘。某幅图像只有在最后一个引用释放后才能回收。现有 warp 需要参考图像，不能直接删掉 `img_` 而不改算法。
5. **输入积压**：点云、IMU、图像订阅深度和本地 deque 独立于 LRU。Valgrind 现为可选 `mapping_prefix`，正常回放默认不启用；检查器下必须降速并观察队列。
6. **轨迹与保存缓存**：`src/LIVMapper.cpp:1447` 的 `path.poses` 持续追加；PCD 缓存也独立于 LRU。当前 `pcd_save_en=false`，所以 `interval=-1` 暂不导致保存缓存积累；以后开启时需分批写盘。显示轨迹可另设抽稀/窗口，完整轨迹写文件。
7. **统计误标（已修复）**：VIO 输出改为 `Visual Map Voxels`，周期 `[Memory]` 同时记录两张地图的容量和淘汰累计数。

因此，“LRU 容量=一百万”是每张地图的体素条目数，不是点数、不是总共一百万，也不是某个固定 GB 的进程内存上限。建议先修确定的泄漏，再结合两张地图的体素数、视觉点/观测/不同图像数、队列长度、RSS 和处理延迟选容量。

## 实施与验收状态

1. patch/Warp、根地图与相机所有权、内存统计已完成。
2. 共享 LRU、两套容量、ROS2 参数、全部访问路径和统一负坐标 key 已完成。
3. 11 个边界/生命周期测试通过；定向 Memcheck 为 0 errors、0 definite/indirect/possible lost。
4. 同一 352.4 秒 bag 的纯 LIO 与 LIVO 无检查器回放完成。LIO 最终为 30000/0 体素、4221 次激光淘汰、约 366 MiB RSS；LIVO 最终为 30000/2000 体素、8911/10290 次淘汰、约 3.17 GiB RSS，进程均运行到 bag 结束。
5. Valgrind 短回放覆盖实际 LIO/VIO 路径；依赖库和非完整 FastDDS 退出仍会产生报告，工程生命周期以定向测试和无非法读写为准。轨迹精度对比和更多路线仍需单独做数据集评估。

淘汰旧体素后，重访该区域需要重新建立前端地图，容量过小可能影响匹配与精度。已经发布或写盘的历史点云不会自动被撤回，但这套在线 LRU 地图本身不再保留完整历史。验收应证明地图条目受控、无非法访问、队列不过度积压且精度可接受，不能只以某一时刻 RSS 下降作为成功标准。
