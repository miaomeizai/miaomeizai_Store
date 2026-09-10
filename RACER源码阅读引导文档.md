# RACER 源码阅读引导文档

> 面向新手的多无人机分布式探索算法源码解析
> 生成日期：2026-09-07

---

## 目录

1. [项目整体介绍](#1-项目整体介绍)
2. [目录结构与各包功能](#2-目录结构与各包功能)
3. [节点与通信拓扑](#3-节点与通信拓扑)
4. [核心模块详解](#4-核心模块详解)
5. [启动流程](#5-启动流程)
6. [推荐阅读顺序](#6-推荐阅读顺序)
7. [后续改造重点标注](#7-后续改造重点标注)
8. [核心文件速查表](#8-核心文件速查表)

---

## 1. 项目整体介绍

### 1.1 项目是什么

RACER（**R**apid **A**utonomous **C**overage via **E**xploration in **R**eal-time）是一个多无人机（Multi-UAV）分布式自主探索系统。它的目标是：让多架无人机在未知环境中协同工作，尽快把整个环境"探索完毕"（即把未知区域都变成已知区域）。

### 1.2 核心技术路线

整个系统可以概括为三层架构：

```
┌─────────────────────────────────────────────────────────┐
│                    决策层（Exploration）                  │
│   前沿提取 → 分层网格分区 → Pair-wise 分配 → 前沿选择     │
└─────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────┐
│                    规划层（Planning）                     │
│   A*路径搜索 → 运动学路径 → B-Spline轨迹优化 → 偏航规划    │
└─────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────┐
│                    执行层（Simulation/Control）            │
│   位置指令 → SO3控制器 → 无人机动力学 → 位姿反馈           │
└─────────────────────────────────────────────────────────┘
```

**关键技术点：**

| 技术 | 说明 |
|------|------|
| **前沿探索（Frontier-based）** | 从地图中提取"已知-未知"边界作为探索目标 |
| **分层网格（HGrid）** | 将地图分成两层均匀网格，用于分布式任务分配 |
| **Pair-wise 分配** | 两架无人机之间协商分配网格，避免全局通信 |
| **人工代价函数** | 基于路径长度、偏航代价等人工设计的代价函数选择最优前沿 |
| **B-Spline 轨迹优化** | 将路径优化为平滑可执行的轨迹 |

### 1.3 仿真模式说明

本项目是**纯算法仿真版本**，不含 Gazebo/PX4 真实物理仿真。具体包括：

- **地图生成器（map_generator）**：从 `.pcd` 文件加载静态点云地图
- **局部感知仿真（local_sensing）**：模拟深度相机的点云输出
- **无人机动力学仿真（so3_quadrotor_simulator）**：模拟四旋翼动力学
- **控制器（so3_control）**：将位置指令转换为推力和力矩
- **可视化（Rviz）**：通过 ROS Marker 展示前沿、网格、轨迹等

---

## 2. 目录结构与各包功能

### 2.1 项目根目录结构

```
RACER/
├── files/                          # 资源文件（pcd地图等）
├── swarm_exploration/              # 【核心】算法包集合
│   ├── exploration_manager/        # 探索管理器（主控节点）
│   ├── active_perception/          # 主动感知（前沿提取、网格分区）
│   ├── plan_env/                   # 规划环境（地图、SDF）
│   ├── plan_manage/                # 规划管理（轨迹服务器、FSM）
│   ├── path_searching/             # 路径搜索（A*、Kinodynamic A*）
│   ├── bspline/                    # B样条基础类
│   ├── bspline_opt/                # B样条优化器
│   ├── poly_traj/                  # 多项式轨迹
│   ├── traj_utils/                 # 轨迹工具与可视化
│   └── utils/                      # 工具包（LKH求解器）
└── uav_simulator/                  # 仿真器包集合
    ├── map_generator/              # 地图生成
    ├── local_sensing/              # 局部感知仿真
    ├── so3_quadrotor_simulator/    # 四旋翼动力学
    ├── so3_control/                # SO3控制器
    ├── poscmd_2_odom/              # 位置指令到位姿
    └── Utils/                      # 工具包集合
```

### 2.2 各包功能说明

#### 核心算法包（swarm_exploration/）

| 包名 | 功能 | 关键文件 | 重要程度 |
|------|------|---------|---------|
| **exploration_manager** | 探索主控：FSM状态机、任务分配、运动规划调度 | `fast_exploration_fsm.cpp`, `fast_exploration_manager.cpp` | ⭐⭐⭐⭐⭐ |
| **active_perception** | 主动感知：前沿提取、分层网格、视点规划 | `frontier_finder.cpp`, `hgrid.cpp`, `uniform_grid.cpp` | ⭐⭐⭐⭐⭐ |
| **plan_env** | 规划环境：占据栅格地图、SDF距离场 | `sdf_map.cpp`, `map_ros.cpp`, `edt_environment.cpp` | ⭐⭐⭐⭐ |
| **plan_manage** | 轨迹管理：轨迹服务器、FSM、轨迹执行 | `traj_server.cpp`, `planner_manager.cpp` | ⭐⭐⭐⭐ |
| **path_searching** | 路径搜索：A*、Kinodynamic A*、Topo PRM | `astar2.cpp`, `kinodynamic_astar.cpp` | ⭐⭐⭐ |
| **bspline** | B样条基础类：非均匀B样条定义 | `non_uniform_bspline.cpp` | ⭐⭐⭐ |
| **bspline_opt** | B样条优化：轨迹平滑优化 | `bspline_optimizer.cpp` | ⭐⭐⭐ |
| **poly_traj** | 多项式轨迹生成 | `traj_generator.cpp` | ⭐⭐ |
| **traj_utils** | 轨迹工具：可视化、消息处理 | `planning_visualization.cpp` | ⭐⭐ |
| **lkh_mtsp_solver** | LKH MTSP求解器：多旅行商问题求解 | `mtsp_node.cpp` | ⭐⭐⭐ |
| **lkh_tsp_solver** | LKH TSP求解器：单旅行商问题求解 | `tsp_node.cpp` | ⭐⭐⭐ |

#### 仿真器包（uav_simulator/）

| 包名 | 功能 | 重要程度 |
|------|------|---------|
| **map_generator** | 从pcd文件生成点云地图 | ⭐⭐ |
| **local_sensing** | 模拟深度相机，生成局部点云和深度图 | ⭐⭐ |
| **so3_quadrotor_simulator** | 四旋翼动力学仿真 | ⭐ |
| **so3_control** | SO3控制器，位置指令→推力/力矩 | ⭐ |
| **poscmd_2_odom** | 位置指令到位姿转换 | ⭐ |
| **odom_visualization** | 无人机模型可视化 | ⭐ |
| **quadrotor_msgs** | 无人机相关消息定义 | ⭐ |

---

## 3. 节点与通信拓扑

### 3.1 核心节点

每个无人机实例化以下节点（以 drone_id=N 为例）：

| 节点名 | 包名 | 功能 |
|--------|------|------|
| `exploration_node_N` | exploration_manager | 探索主控：FSM、前沿、分配、规划调度 |
| `traj_server_N` | plan_manage | 轨迹服务器：B样条→位置指令 |
| `map_pub` | map_generator | 地图发布（全局唯一） |
| `tsp_solver_N` | lkh_mtsp_solver | MTSP求解器服务 |
| `acvrp_solver_N` | lkh_mtsp_solver | ACVRP求解器服务 |
| `pcl_render_node_N` | local_sensing | 局部感知仿真 |
| `poscmd_2_odom_N` | poscmd_2_odom | 指令→位姿转换 |
| `so3_control_N` | so3_control | 控制器 |
| `quadrotor_simulator_N` | so3_quadrotor_simulator | 动力学仿真 |

### 3.2 话题通信拓扑

```
【全局话题】
/map_generator/pcl_render_node/cloud_1      ← 点云地图
/move_base_simple/goal                       ← 触发探索（Rviz 2D Nav Goal）

【每架无人机独立话题】（以 drone_id=1 为例）
/odom_world_1                     ← 位姿反馈
/pcl_render_node/depth_1          ← 深度图
/pcl_render_node/sensor_pose_1    ← 相机位姿
/planning/bspline_1               ← B样条轨迹
/planning/replan_1                ← 重规划信号
/planning/new_1                   ← 新轨迹信号
/planning/pos_cmd_1               ← 位置指令

【集群通信话题】（所有无人机共享）
/swarm_expl/drone_state           ← 无人机状态广播
/swarm_expl/pair_opt              ← Pair-wise 分配请求
/swarm_expl/pair_opt_res          ← Pair-wise 分配响应
/planning/swarm_traj              ← 轨迹广播（避碰用）
```

### 3.3 数据流向

```
传感器数据 → 地图构建 → 前沿提取 → 任务分配 → 路径规划 → 轨迹优化 → 控制执行
   ↓           ↓           ↓           ↓           ↓           ↓
深度图/点云  占据栅格    Frontier    HGrid分配   A*/KinoA*    B-Spline优化
                     + viewpoints  (Pair-wise)              + 偏航规划
```

**详细流程：**

1. **感知阶段**：`pcl_render_node` 根据无人机当前位姿，从全局点云地图中渲染出局部深度图和点云
2. **建图阶段**：`exploration_node` 中的 `MapROS` 接收深度图/点云，更新 `SDFMap` 占据栅格
3. **前沿提取**：`FrontierFinder::searchFrontiers()` 在更新的栅格区域中搜索前沿体素
4. **分区分配**：`HGrid` 将地图分成两层网格，无人机通过 `optTimerCallback` 协商分配
5. **路径规划**：`FastExplorationManager::findGridAndFrontierPath()` 规划网格级和前沿级路径
6. **轨迹优化**：`planner_manager_` 将路径优化为 B-Spline 轨迹
7. **控制执行**：`traj_server` 解析轨迹为位置指令，`so3_control` 执行控制

---

## 4. 核心模块详解

### 4.1 地图构建与占据栅格管理

**功能**：维护一个三维占据栅格地图，支持概率更新和SDF距离场计算。

**关键文件**：
- `RACER/swarm_exploration/plan_env/src/sdf_map.cpp` — 占据栅格地图核心实现
- `RACER/swarm_exploration/plan_env/src/map_ros.cpp` — ROS接口，处理传感器输入
- `RACER/swarm_exploration/plan_env/include/plan_env/sdf_map.h` — 地图类定义

**核心类**：
- `SDFMap`：占据栅格地图，支持UNKNOWN/FREE/OCCUPIED三种状态
- `MapROS`：ROS接口层，处理深度图和点云输入
- `MultiMapManager`：多无人机地图块管理

**输入输出**：
- 输入：深度图 (`/pcl_render_node/depth_N`)、相机位姿 (`/pcl_render_node/sensor_pose_N`)
- 输出：占据栅格、SDF距离场

**核心逻辑**：
1. 接收深度图，通过相机内参反投影为三维点云
2. 使用射线投射（ray casting）更新射线经过的栅格
3. 基于概率模型更新占据概率（p_hit=0.65, p_miss=0.35）
4. 计算ESDF（欧几里得符号距离场）用于轨迹优化
5. 维护局部更新区域，用于前沿提取的增量搜索

**关键参数**：
- `sdf_map/resolution`：栅格分辨率（默认0.1m）
- `sdf_map/map_size_x/y/z`：地图尺寸
- `sdf_map/obstacles_inflation`：障碍物膨胀半径

---

### 4.2 前沿提取模块 ⭐⭐⭐⭐⭐

**功能**：从占据栅格地图中自动提取"已知-未知"边界（frontier），并为每个前沿计算最优视点。

**关键文件**：
- `RACER/swarm_exploration/active_perception/src/frontier_finder.cpp` — 前沿提取核心实现
- `RACER/swarm_exploration/active_perception/include/active_perception/frontier_finder.h` — 类定义

**核心类**：
- `FrontierFinder`：前沿管理器
- `Frontier`：单个前沿簇，包含体素、包围盒、视点等
- `Viewpoint`：覆盖某个前沿簇的视点（位置+偏航）

**输入输出**：
- 输入：`SDFMap` 占据栅格、当前无人机位置
- 输出：前沿簇列表、每个前沿的最优视点、前沿间代价矩阵

**核心逻辑**（`searchFrontiers()`）：

1. **获取更新区域**：从 `SDFMap` 获取本次更新的包围盒
2. **清除已变化的前沿**：检查现有前沿是否因地图更新而改变，移除已变化的
3. **搜索新前沿种子**：在更新区域内扫描，找到"已知空闲且邻居有未知"的体素
4. **区域生长聚类**：从种子体素开始BFS扩展，形成完整的前沿簇
5. **分裂过大前沿**：对超过尺寸阈值的前沿沿主成分方向分裂
6. **计算视点**：为每个前沿采样候选视点，计算可见体素数，选择最优视点

**前沿判定条件**（`isNeighborUnknown()`）：
- 当前体素为已知空闲（FREE）
- 6/10/26邻居中至少有一个是未知（UNKNOWN）

**关键参数**（`frontier/*`）：
- `cluster_min`：最小前沿簇体素数（100）
- `cluster_size_xy/z`：前沿簇最大尺寸
- `candidate_rmin/rmax`：视点采样距离范围

---

### 4.3 任务分配模块 ⭐⭐⭐⭐⭐ 【毕设改造重点】

**功能**：将探索区域（网格）分配给多架无人机，最小化总探索代价。

**关键文件**：
- `RACER/swarm_exploration/active_perception/src/hgrid.cpp` — 分层网格实现
- `RACER/swarm_exploration/active_perception/include/active_perception/hgrid.h` — 类定义
- `RACER/swarm_exploration/active_perception/src/uniform_grid.cpp` — 均匀网格实现
- `RACER/swarm_exploration/exploration_manager/src/fast_exploration_manager.cpp` — 分配算法
- `RACER/swarm_exploration/exploration_manager/src/fast_exploration_fsm.cpp` — Pair-wise协商

**核心类**：
- `HGrid`：分层网格管理器（两层：Level 1粗网格、Level 2细网格）
- `UniformGrid`：单层均匀网格
- `GridInfo`：单个网格信息（未知数、前沿数、包围盒等）

**分配机制详解**：

#### （1）分层网格构建

```
Level 1（粗网格）：将地图分成较大的均匀网格
     ↓ 当网格内未知区域过多时
Level 2（细网格）：将粗网格进一步细分
```

每个网格维护：
- `unknown_num_`：未知体素数量
- `frontier_num_`：包含的前沿数
- `center_`：网格中心坐标
- `active_`：是否激活（有前沿则激活）

#### （2）Pair-wise 分配流程 【震荡产生处】

**触发条件**（`optTimerCallback`）：
- 当前无人机没有分配到网格（`grid_ids.empty()`）
- 距离上次尝试超过 `attempt_interval_`（0.1s）
- 选择距离最近且长时间未交互的无人机

**协商流程**：

```
Drone A                                    Drone B
   │                                          │
   │  1. 收集双方的 grid_ids 并集              │
   │  2. 计算 ACVRP 问题文件                   │
   │  3. 调用 LKH 求解器                      │
   │  4. 生成分配结果                          │
   │───────── PairOpt 消息 ─────────────────→│
   │                                          │  5. 检查时间戳避免频繁修改
   │                                          │  6. 接受或拒绝分配
   │←──────── PairOptResponse 消息 ──────────│
   │  7. 验证响应并更新本地分配                 │
```

**ACVRP 问题建模**：
- 节点：两架无人机当前位置 + 各网格中心
- 代价矩阵：`HGrid::getCostMatrix()` 计算节点间路径代价
- 容量约束：基于网格内未知体素数量
- 求解器：LKH-3 求解带容量约束的车辆路径问题（ACVRP）

**震荡产生原因**：
1. **时间戳竞争**：两架无人机同时发起分配请求
2. **局部最优**：ACVRP 求解结果不稳定，每次分配可能不同
3. **频繁重分配**：`pair_opt_interval_` 设置过短导致频繁协商
4. **状态不一致**：网格状态在协商过程中发生变化

**关键参数**：
- `fsm/attempt_interval`：最小尝试间隔（0.1s）
- `fsm/pair_opt_interval`：最小交互间隔（0.5s）
- `partitioning/consistent_cost`：一致性代价权重
- `partitioning/grid_size`：网格大小（5.0m）

---

### 4.4 前沿选择模块 ⭐⭐⭐⭐⭐ 【毕设改造重点】

**功能**：为每架无人机选择最优的探索前沿，基于人工代价函数排序。

**关键文件**：
- `RACER/swarm_exploration/active_perception/src/graph_node.cpp` — 代价计算核心
- `RACER/swarm_exploration/exploration_manager/src/fast_exploration_manager.cpp` — 选择逻辑
- `RACER/swarm_exploration/exploration_manager/src/fast_exploration_fsm.cpp` — 调用入口

**核心函数**：
- `ViewNode::computeCost()`：人工代价函数（**核心**）
- `FastExplorationManager::findGridAndFrontierPath()`：网格级+前沿级路径规划
- `FastExplorationManager::findTourOfFrontier()`：前沿级TSP求解
- `FastExplorationManager::refineLocalTour()`：局部视点优化

**代价函数公式**（`ViewNode::computeCost()`）：

```cpp
double computeCost(p1, p2, y1, y2, v1, yd1, path) {
    // 1. 位置代价：A*搜索路径长度 / 最大速度
    double pos_cost = searchPath(p1, p2, path) / vm_;

    // 2. 速度方向代价：当前速度方向与目标方向的夹角
    if (v1.norm() > 1e-3) {
        Vector3d dir = (p2 - p1).normalized();
        Vector3d vdir = v1.normalized();
        double diff = acos(vdir.dot(dir));
        pos_cost += w_dir_ * diff;  // w_dir_ 默认 1.5
    }

    // 3. 偏航代价：目标偏航角与当前偏航角的差值 / 最大偏航速率
    double diff = fabs(y2 - y1);
    diff = min(diff, 2 * M_PI - diff);
    double yaw_cost = diff / yd_;

    // 4. 总代价：取位置代价和偏航代价的最大值
    return max(pos_cost, yaw_cost);
}
```

**选择流程**：

1. **网格级规划**（`findGridAndFrontierPath`）：
   - 调用 `findGlobalTourOfGrid()` 获取分配的网格序列
   - 调用 `hgrid_->getFrontiersInGrid()` 获取当前网格内的前沿

2. **前沿级规划**（`findTourOfFrontier`）：
   - 构建前沿间代价矩阵（`getFullCostMatrix`）
   - 调用 LKH TSP 求解器获取最优访问序列
   - 返回前沿 ID 列表

3. **视点精炼**（`refineLocalTour`）：
   - 为前 K 个前沿获取 Top-N 候选视点
   - 构建图搜索问题，用 Dijkstra 求解最优视点序列
   - 返回下一个目标视点（位置+偏航）

**关键参数**：
- `exploration/vm`：最大速度（1.5 m/s）
- `exploration/am`：最大加速度（1.0 m/s²）
- `exploration/yd`：最大偏航速率（80°/s）
- `exploration/w_dir`：速度方向代价权重（1.5）
- `exploration/refined_num`：局部精炼的前沿数（7）
- `exploration/top_view_num`：每个前沿的候选视点数（15）

---

### 4.5 局部路径规划模块

**功能**：将探索决策转化为可执行的平滑轨迹。

**关键文件**：
- `RACER/swarm_exploration/plan_manage/src/planner_manager.cpp` — 规划管理器
- `RACER/swarm_exploration/path_searching/src/astar2.cpp` — A*路径搜索
- `RACER/swarm_exploration/path_searching/src/kinodynamic_astar.cpp` — 运动学A*
- `RACER/swarm_exploration/bspline_opt/src/bspline_optimizer.cpp` — B样条优化
- `RACER/swarm_exploration/poly_traj/src/traj_generator.cpp` — 多项式轨迹生成

**核心类**：
- `FastPlannerManager`：规划管理器，协调各模块
- `Astar`：A*路径搜索
- `KinodynamicAstar`：运动学约束A*搜索
- `BsplineOptimizer`：B样条轨迹优化器

**规划流程**：

```
目标视点 (pos, yaw)
       ↓
┌──────────────────────────┐
│ 1. A*几何路径搜索         │  ← 找到无碰撞的折线路径
│    astar2.cpp             │
└──────────────────────────┘
       ↓
┌──────────────────────────┐
│ 2. 路径缩短              │  ← 移除冗余中间点
│    shortenPath()         │
└──────────────────────────┘
       ↓
┌──────────────────────────┐
│ 3. B-Spline轨迹优化       │  ← 优化为平滑可执行轨迹
│    bspline_optimizer.cpp │     考虑：平滑性、可行性、
└──────────────────────────┘     障碍物距离、集群避碰
       ↓
┌──────────────────────────┐
│ 4. 偏航轨迹规划           │  ← 独立规划偏航角
│    planYawExplore()      │
└──────────────────────────┘
       ↓
   B-Spline 轨迹输出
```

**轨迹优化目标函数**（`BsplineOptimizer`）：
- 平滑性代价：轨迹的jerk最小化
- 可行性代价：满足速度/加速度约束
- 障碍物代价：与障碍物保持安全距离
- 集群避碰代价：与其他无人机轨迹保持距离

---

### 4.6 FSM 状态机

**功能**：管理探索流程的状态转换。

**关键文件**：
- `RACER/swarm_exploration/exploration_manager/src/fast_exploration_fsm.cpp`
- `RACER/swarm_exploration/exploration_manager/include/exploration_manager/fast_exploration_fsm.h`

**状态定义**：
```cpp
enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };
```

**状态转换图**：

```
                    ┌─────────────────┐
                    │      INIT       │
                    │  等待里程计就绪   │
                    └────────┬────────┘
                             ↓
                    ┌─────────────────┐
                    │  WAIT_TRIGGER   │
                    │  等待Rviz触发    │
                    └────────┬────────┘
                             ↓ (trigger)
                    ┌─────────────────┐
               ┌───→│    PLAN_TRAJ    │←──────────────────┐
               │    │   规划探索轨迹    │                   │
               │    └────────┬────────┘                   │
               │             ↓ (SUCCEED)                  │
               │    ┌─────────────────┐                   │
               │    │    PUB_TRAJ     │                   │
               │    │   发布轨迹       │                   │
               │    └────────┬────────┘                   │
               │             ↓                            │
               │    ┌─────────────────┐                   │
               │    │    EXEC_TRAJ    │───────────────────┘
               │    │   执行轨迹       │  (replan)
               │    └────────┬────────┘
               │             ↓ (无前沿/完成)
               │    ┌─────────────────┐
               │    │      IDLE       │←── (100s无前沿)
               │    │   空闲状态       │──→ FINISH
               │    └─────────────────┘
               └─────────────────────────────
```

---

## 5. 启动流程

### 5.1 主 Launch 文件

**主 Launch 文件**：`RACER/swarm_exploration/exploration_manager/launch/swarm_exploration.launch`

**启动内容**：
1. 定义地图尺寸（默认 35x35x3.5m）
2. 定义无人机数量（默认 5 架）
3. 启动地图生成器（加载 `pillar.pcd`）
4. 为每架无人机包含 `single_drone_exploration.xml`

### 5.2 单无人机启动链路

```
swarm_exploration.launch
    └── single_drone_exploration.xml
            ├── single_drone_planner.xml
            │       ├── exploration_node (exploration_manager)
            │       │       ├── MapROS (地图构建)
            │       │       ├── FrontierFinder (前沿提取)
            │       │       ├── HGrid (分层网格)
            │       │       ├── FastExplorationManager (探索管理)
            │       │       └── FastExplorationFSM (状态机)
            │       ├── tsp_solver (lkh_tsp_solver)
            │       └── acvrp_solver (lkh_mtsp_solver)
            ├── traj_server (plan_manage)
            └── simulator_light.xml (仿真器)
                    ├── pcl_render_node (local_sensing)
                    ├── poscmd_2_odom
                    ├── so3_control
                    └── quadrotor_simulator
```

### 5.3 关键配置参数位置

| 参数类别 | 参数路径 | 说明 |
|---------|---------|------|
| **无人机数量** | `swarm_exploration.launch: drone_num` | 默认5 |
| **地图尺寸** | `swarm_exploration.launch: map_size_x/y/z` | 35x35x3.5m |
| **地图文件** | `swarm_exploration.launch: map_pub args` | pillar.pcd |
| **栅格分辨率** | `single_drone_planner.xml: sdf_map/resolution` | 0.1m |
| **前沿参数** | `single_drone_planner.xml: frontier/*` | cluster_min=100等 |
| **分配参数** | `single_drone_planner.xml: partitioning/*` | grid_size=5.0等 |
| **探索参数** | `single_drone_planner.xml: exploration/*` | vm, am, yd等 |
| **规划参数** | `single_drone_planner.xml: manager/*` | max_vel, max_acc等 |

---

## 6. 推荐阅读顺序

### 阶段一：整体理解（1-2天）

**目标**：理解系统架构和数据流，不深究细节

| 文件 | 阅读重点 | 重要程度 |
|------|---------|---------|
| `exploration_manager/launch/swarm_exploration.launch` | 启动流程、参数位置 | 了解 |
| `exploration_manager/include/exploration_manager/expl_data.h` | 核心数据结构定义 | 精读 |
| `exploration_manager/src/exploration_node.cpp` | 主程序入口 | 了解 |
| `exploration_manager/include/exploration_manager/fast_exploration_fsm.h` | FSM状态定义 | 精读 |

### 阶段二：核心流程（3-5天）

**目标**：理解探索的完整流程

| 文件 | 阅读重点 | 重要程度 |
|------|---------|---------|
| `exploration_manager/src/fast_exploration_fsm.cpp` | FSM实现、回调函数 | 精读 |
| `exploration_manager/src/fast_exploration_manager.cpp` | 探索管理、分配逻辑 | 精读 |
| `active_perception/src/frontier_finder.cpp` | 前沿提取算法 | 精读 |

### 阶段三：任务分配（2-3天）⭐毕设重点

**目标**：深入理解分配机制，为替换做准备

| 文件 | 阅读重点 | 重要程度 |
|------|---------|---------|
| `active_perception/src/hgrid.cpp` | 分层网格管理 | 精读 |
| `active_perception/src/uniform_grid.cpp` | 单层网格实现 | 精读 |
| `active_perception/include/active_perception/hgrid.h` | HGrid接口定义 | 精读 |
| `exploration_manager/msg/PairOpt.msg` | Pair-wise消息定义 | 精读 |
| `exploration_manager/msg/DroneState.msg` | 无人机状态消息 | 了解 |

### 阶段四：代价函数（1-2天）⭐毕设重点

**目标**：理解人工代价函数，为DQN替换做准备

| 文件 | 阅读重点 | 重要程度 |
|------|---------|---------|
| `active_perception/src/graph_node.cpp` | 代价函数computeCost | 精读 |
| `active_perception/include/active_perception/graph_node.h` | ViewNode定义 | 精读 |
| `active_perception/src/graph_search.h` | 图搜索模板 | 了解 |
| `path_searching/src/astar2.cpp` | A*路径搜索 | 了解 |

### 阶段五：轨迹规划（2-3天）

**目标**：理解轨迹生成和优化

| 文件 | 阅读重点 | 重要程度 |
|------|---------|---------|
| `plan_manage/src/planner_manager.cpp` | 规划管理器 | 了解 |
| `bspline_opt/src/bspline_optimizer.cpp` | B样条优化 | 了解 |
| `plan_manage/src/traj_server.cpp` | 轨迹服务器 | 了解 |
| `plan_env/src/sdf_map.cpp` | 地图管理 | 了解 |

### 阶段六：辅助模块（1天）

**目标**：了解仿真器和工具

| 文件 | 阅读重点 | 重要程度 |
|------|---------|---------|
| `uav_simulator/local_sensing/src/` | 局部感知仿真 | 了解 |
| `uav_simulator/so3_control/src/` | 控制器 | 了解 |
| `utils/lkh_mtsp_solver/` | LKH求解器接口 | 了解 |

---

## 7. 后续改造重点标注

### 7.1 替换 Pair-wise 分配 → 加权 Voronoi 动态分区

**需要修改的文件**：

| 文件 | 修改内容 | 插入点 |
|------|---------|--------|
| `exploration_manager/src/fast_exploration_fsm.cpp` | 修改 `optTimerCallback()` | 第715-854行 |
| `exploration_manager/src/fast_exploration_manager.cpp` | 修改 `allocateGrids()` | 第658-799行 |
| `active_perception/src/hgrid.cpp` | 修改或替换 `updateGridData()` | 第71-155行 |
| `active_perception/src/uniform_grid.cpp` | 修改 `getCostMatrix()` | 成本矩阵计算 |
| `exploration_manager/include/exploration_manager/expl_data.h` | 修改 `DroneState` 结构 | 添加Voronoi相关字段 |

**核心插入点**：

```
optTimerCallback()          ← Pair-wise 协商入口
    ↓
allocateGrids()             ← 分配算法入口（ACVRP求解）
    ↓
hgrid_->getCostMatrix()     ← 代价矩阵计算
    ↓
hgrid_->updateGridData()    ← 网格状态更新
```

**可复用模块**：
- `FrontierFinder`：前沿提取完全复用
- `plan_env`：地图管理完全复用
- `bspline_opt`：轨迹优化完全复用
- `path_searching`：路径搜索完全复用

### 7.2 替换人工代价函数 → DQN 前沿优先级选择

**需要修改的文件**：

| 文件 | 修改内容 | 插入点 |
|------|---------|--------|
| `active_perception/src/graph_node.cpp` | 修改 `computeCost()` | 第63-99行 |
| `exploration_manager/src/fast_exploration_manager.cpp` | 修改 `findTourOfFrontier()` | 前沿排序逻辑 |
| `exploration_manager/src/fast_exploration_manager.cpp` | 修改 `refineLocalTour()` | 视点选择逻辑 |
| `exploration_manager/include/exploration_manager/expl_data.h` | 添加DQN相关数据结构 | 新增字段 |

**核心插入点**：

```
findGridAndFrontierPath()    ← 规划入口
    ↓
findTourOfFrontier()         ← 前沿TSP规划（替换排序逻辑）
    ↓
ViewNode::computeCost()      ← 代价计算（替换为DQN输出）
    ↓
refineLocalTour()            ← 视点精炼（可选替换）
```

**可复用模块**：
- `HGrid`：网格分区完全复用
- `FrontierFinder`：前沿提取完全复用
- `Astar`：A*路径搜索完全复用（用于DQN状态计算）
- `BsplineOptimizer`：轨迹优化完全复用

### 7.3 完全不用动的模块

| 模块 | 原因 |
|------|------|
| `plan_env` (地图管理) | 纯基础设施，与分配和选择无关 |
| `bspline` / `bspline_opt` | 轨迹表示和优化，独立于决策层 |
| `poly_traj` | 多项式轨迹生成，独立于决策层 |
| `path_searching` (A*) | 路径搜索，可直接复用 |
| `traj_utils` | 可视化工具，无需修改 |
| `uav_simulator` | 仿真器，无需修改 |
| `lkh_*_solver` | 如果用DQN替换TSP，可不再使用 |

---

## 8. 核心文件速查表

### 探索管理

| 文件 | 功能 | 路径 |
|------|------|------|
| `exploration_node.cpp` | 主程序入口 | `RACER/swarm_exploration/exploration_manager/src/exploration_node.cpp` |
| `fast_exploration_fsm.cpp` | FSM状态机实现 | `RACER/swarm_exploration/exploration_manager/src/fast_exploration_fsm.cpp` |
| `fast_exploration_fsm.h` | FSM类定义 | `RACER/swarm_exploration/exploration_manager/include/exploration_manager/fast_exploration_fsm.h` |
| `fast_exploration_manager.cpp` | 探索管理器实现 | `RACER/swarm_exploration/exploration_manager/src/fast_exploration_manager.cpp` |
| `fast_exploration_manager.h` | 探索管理器定义 | `RACER/swarm_exploration/exploration_manager/include/exploration_manager/fast_exploration_manager.h` |
| `expl_data.h` | 核心数据结构 | `RACER/swarm_exploration/exploration_manager/include/exploration_manager/expl_data.h` |

### 前沿提取

| 文件 | 功能 | 路径 |
|------|------|------|
| `frontier_finder.cpp` | 前沿提取实现 | `RACER/swarm_exploration/active_perception/src/frontier_finder.cpp` |
| `frontier_finder.h` | 前沿提取定义 | `RACER/swarm_exploration/active_perception/include/active_perception/frontier_finder.h` |
| `graph_node.cpp` | 代价函数实现 | `RACER/swarm_exploration/active_perception/src/graph_node.cpp` |
| `graph_node.h` | ViewNode定义 | `RACER/swarm_exploration/active_perception/include/active_perception/graph_node.h` |

### 任务分配

| 文件 | 功能 | 路径 |
|------|------|------|
| `hgrid.cpp` | 分层网格实现 | `RACER/swarm_exploration/active_perception/src/hgrid.cpp` |
| `hgrid.h` | 分层网格定义 | `RACER/swarm_exploration/active_perception/include/active_perception/hgrid.h` |
| `uniform_grid.cpp` | 均匀网格实现 | `RACER/swarm_exploration/active_perception/src/uniform_grid.cpp` |
| `uniform_grid.h` | 均匀网格定义 | `RACER/swarm_exploration/active_perception/include/active_perception/uniform_grid.h` |

### 地图管理

| 文件 | 功能 | 路径 |
|------|------|------|
| `sdf_map.cpp` | 占据栅格地图 | `RACER/swarm_exploration/plan_env/src/sdf_map.cpp` |
| `sdf_map.h` | 地图类定义 | `RACER/swarm_exploration/plan_env/include/plan_env/sdf_map.h` |
| `map_ros.cpp` | ROS地图接口 | `RACER/swarm_exploration/plan_env/src/map_ros.cpp` |
| `edt_environment.cpp` | EDT环境 | `RACER/swarm_exploration/plan_env/src/edt_environment.cpp` |

### 路径规划

| 文件 | 功能 | 路径 |
|------|------|------|
| `astar2.cpp` | A*搜索 | `RACER/swarm_exploration/path_searching/src/astar2.cpp` |
| `kinodynamic_astar.cpp` | 运动学A* | `RACER/swarm_exploration/path_searching/src/kinodynamic_astar.cpp` |
| `planner_manager.cpp` | 规划管理器 | `RACER/swarm_exploration/plan_manage/src/planner_manager.cpp` |
| `bspline_optimizer.cpp` | B样条优化 | `RACER/swarm_exploration/bspline_opt/src/bspline_optimizer.cpp` |
| `traj_server.cpp` | 轨迹服务器 | `RACER/swarm_exploration/plan_manage/src/traj_server.cpp` |

### Launch 文件

| 文件 | 功能 | 路径 |
|------|------|------|
| `swarm_exploration.launch` | 主启动文件 | `RACER/swarm_exploration/exploration_manager/launch/swarm_exploration.launch` |
| `single_drone_exploration.xml` | 单无人机启动 | `RACER/swarm_exploration/exploration_manager/launch/single_drone_exploration.xml` |
| `single_drone_planner.xml` | 算法参数配置 | `RACER/swarm_exploration/exploration_manager/launch/single_drone_planner.xml` |

### 消息定义

| 文件 | 功能 | 路径 |
|------|------|------|
| `PairOpt.msg` | Pair-wise分配请求 | `RACER/swarm_exploration/exploration_manager/msg/PairOpt.msg` |
| `PairOptResponse.msg` | 分配响应 | `RACER/swarm_exploration/exploration_manager/msg/PairOptResponse.msg` |
| `DroneState.msg` | 无人机状态 | `RACER/swarm_exploration/exploration_manager/msg/DroneState.msg` |
| `Bspline.msg` | B样条轨迹 | `RACER/swarm_exploration/bspline/msg/Bspline.msg` |

---

## 附录：常见术语对照

| 英文术语 | 中文含义 | 在代码中的位置 |
|---------|---------|--------------|
| Frontier | 前沿（已知-未知边界） | `FrontierFinder` |
| Viewpoint | 视点（观察位置+朝向） | `Viewpoint` 结构体 |
| HGrid | 分层网格 | `HGrid` 类 |
| Pair-wise Optimization | 配对优化 | `optTimerCallback()` |
| ACVRP | 带容量约束的车辆路径问题 | `allocateGrids()` |
| TSP | 旅行商问题 | `findGlobalTour()` |
| AmTSP | 不对称多旅行商问题 | LKH求解器 |
| B-Spline | B样条曲线 | `NonUniformBspline` |
| SDF | 符号距离场 | `SDFMap` |
| ESDF | 欧几里得符号距离场 | `EDTEnvironment` |
| FSM | 有限状态机 | `FastExplorationFSM` |

---

**文档完成**。这份文档覆盖了 RACER 项目的核心架构和关键模块，特别标注了后续毕设改造需要重点关注的 Pair-wise 分配和人工代价函数模块。建议按照推荐的阅读顺序逐步深入，先建立整体理解，再深入细节。
