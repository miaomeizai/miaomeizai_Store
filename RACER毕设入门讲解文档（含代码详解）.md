# RACER 毕设入门讲解文档（含代码详解）

> **写给零基础的你**：本文假设你只有最基本的计算机知识（知道什么是程序、什么是文件），对无人机、集群算法、机器人操作系统完全不了解。我们将按照推荐的阅读顺序，结合具体代码，一步一步带你理解这个项目。

---

## 目录

- [第零章：预备知识——你需要知道的基本概念](#第零章预备知识你需要知道的基本概念)
- [第一阶段：整体理解（1-2天）](#第一阶段整体理解1-2天)
  - [1.1 这个项目在做什么？——用一个比喻来理解](#11-这个项目在做什么用一个比喻来理解)
  - [1.2 ROS 是什么？——无人机的"操作系统"](#12-ros-是什么无人机的操作系统)
  - [1.3 项目的"骨架"——目录结构](#13-项目的骨架目录结构)
  - [1.4 启动文件怎么看——swarm_exploration.launch](#14-启动文件怎么看swarm_explorationlaunch)
  - [1.5 核心数据结构——expl_data.h（逐行讲解）](#15-核心数据结构expl_datah逐行讲解)
- [第二阶段：核心流程（3-5天）](#第二阶段核心流程3-5天)
  - [2.1 程序入口——exploration_node.cpp](#21-程序入口exploration_nodecpp)
  - [2.2 有限状态机（FSM）——无人机的"大脑开关"](#22-有限状态机fsm无人机的大脑开关)
  - [2.3 前沿提取——找到"未知的边界"（代码详解）](#23-前沿提取找到未知的边界代码详解)
- [第三阶段：任务分配（2-3天）⭐毕设重点](#第三阶段任务分配2-3天毕设重点)
  - [3.1 分层网格——把地图切成"蛋糕"](#31-分层网格把地图切成蛋糕)
  - [3.2 Pair-wise 分配——两两"商量"](#32-pair-wise-分配两两商量)
- [第四阶段：代价函数（1-2天）⭐毕设重点](#第四阶段代价函数1-2天毕设重点)
  - [4.1 什么是代价函数？——用"打分"来理解](#41-什么是代价函数用打分来理解)
  - [4.2 RACER 的代价函数——graph_node.cpp 逐行讲解](#42-racer-的代价函数graph_nodecpp-逐行讲解)
- [第五阶段：轨迹规划（2-3天）](#第五阶段轨迹规划2-3天)
  - [5.1 探索管理器如何规划——fast_exploration_manager.cpp 详解](#51-探索管理器如何规划fast_exploration_managercpp-详解)
- [附录A：常见术语中英文对照](#附录a常见术语中英文对照)
- [附录B：你的毕设改造路线图](#附录b你的毕设改造路线图)

---

## 第零章：预备知识——你需要知道的基本概念

在开始读代码之前，先搞懂这几个词，后面就不会卡住。

### 0.1 什么是无人机集群（Swarm）？

> **一句话**：一群无人机一起干活，而不是一架单独干。

想象一下：一个人打扫一栋大楼很慢，但如果派 10 个人同时从不同楼层开始打扫，效率就高多了。无人机集群就是这个道理——多架无人机分工合作，共同完成一个任务。

### 0.2 什么是"主动探索"（Active Exploration）？

> **一句话**：无人机自己决定"下一步去哪里看"。

普通的无人机需要人遥控，但"主动探索"意味着无人机自己做决定：我周围哪里还没看过？我应该先去看哪里？这就是"主动"的意思——不是随机乱飞，而是有策略地选择最有价值的方向。

### 0.3 什么是 ROS？

> **一句话**：一套让机器人各模块之间能互相"说话"的软件框架。

ROS = **R**obot **O**perating **S**ystem（机器人操作系统）。它不是真正的操作系统（不是 Windows、Linux 那种），而是一套**通信工具**。

**打个比方**：
- 你有一个团队：有人负责看路（感知），有人负责算路线（规划），有人负责开车（控制）
- ROS 就是给每个人配了一部对讲机，让他们可以互相发消息
- 每个对讲机频道有一个名字，比如 `/drone1/camera`（无人机1号的摄像头数据）
- 这些频道在 ROS 里叫做**话题（Topic）**

### 0.4 什么是 Launch 文件？

> **一句话**：一个"一键启动"脚本，告诉 ROS 同时启动哪些程序。

就像你双击一个快捷方式可以同时打开好几个软件一样，Launch 文件（`.launch` 或 `.xml`）告诉 ROS："帮我同时启动这些节点，设置好这些参数。"

### 0.5 什么是节点（Node）？

> **一句话**：一个独立运行的小程序。

在 ROS 里，每个功能都是一个独立的小程序，叫做"节点"。比如：
- 一个节点专门负责"发布地图数据"
- 一个节点专门负责"计算飞行路线"
- 一个节点专门负责"控制无人机飞"

这些节点通过 ROS 的话题（Topic）互相通信。

### 0.6 什么是点云地图和占据栅格？

> **点云（Point Cloud）**：用很多很多个三维空间中的点来描述环境的形状。就像用乐高积木搭建一个场景，每个积木块的位置就是一个"点"。
>
> **占据栅格（Occupancy Grid）**：把空间划分成一个个小格子（立方体），每个格子标记为"空的"、"有障碍物"、或"不知道"。就像一张像素画，每个像素标记颜色。

### 0.7 什么是 B-Spline（B样条）？

> **一句话**：一种数学工具，可以把几个控制点变成一条平滑的曲线。

想象你手里有几个点（控制点），你想用一条平滑的曲线把它们连起来，让无人机能顺畅地飞过去而不是急转弯。B-Spline 就是做这个的。

---

## 第一阶段：整体理解（1-2天）

> **目标**：不需要看懂每一行代码，只需要搞清楚"整个系统长什么样"、"数据从哪里来到哪里去"。

### 1.1 这个项目在做什么？——用一个比喻来理解

**场景**：你被扔进一个陌生的、漆黑的大楼里，你的任务是把整栋楼探索完（每个房间都进去看一眼）。

**如果只有一架无人机**：
1. 手里拿着手电筒（传感器），只能照亮周围一小片区域
2. 每照亮一片，就把已知的地图更新一下
3. 看看地图上哪里还有"黑暗区域"（未知区域）
4. 找到已知和未知的交界处（这就是"前沿"）
5. 飞到最近的前沿去照亮它
6. 重复，直到整栋楼都被照亮

**如果有 5 架无人机（集群）**：
- 问题变成：5 架飞机怎么分工，才能最快地把整栋楼探索完？
- 需要一个"分配方案"：谁去哪个区域？
- 需要"互相避碰"：别撞到一起
- 需要"通信协调"：知道队友在哪里、在干什么

**RACER 就是解决这个问题的算法系统。**

### 1.2 ROS 是什么？——无人机的"操作系统"

> 前面在 0.3 已经解释过了，这里补充一点实际的。

在 RACER 项目里，ROS 的使用体现在：

1. **话题通信**：无人机的各个模块通过"话题"（Topic）传递数据
   - 比如 `/pcl_render_node/depth_1` 这个话题传递的是"1号无人机的深度相机数据"
   - 比如 `/odom_world_1` 传递的是"1号无人机的位置和姿态"

2. **参数服务器**：一些配置参数（比如地图大小、无人机数量）通过 ROS 的参数服务器传递

3. **Launch 文件**：启动整个系统

> **你现在不需要深入学 ROS**，只需要知道：代码里那些 `ros::Publisher`、`ros::Subscriber`、`ros::NodeHandle` 都是 ROS 的通信接口，作用就是"发布消息"和"订阅消息"。

### 1.3 项目的"骨架"——目录结构

打开项目文件夹，你会看到：

```
RACER/
├── files/                          ← 存放地图文件（.pcd 格式）
├── swarm_exploration/              ← 【最重要】所有算法代码在这里
│   ├── exploration_manager/        ← "总指挥部"：管理整个探索流程
│   ├── active_perception/          ← "眼睛和大脑"：找前沿、分区域
│   ├── plan_env/                   ← "地图"：记住哪里有障碍物
│   ├── plan_manage/                ← "调度中心"：管理轨迹的生成和执行
│   ├── path_searching/             ← "导航员"：找从A到B的路
│   ├── bspline/                    ← "画线工具"：B样条的基础定义
│   ├── bspline_opt/                ← "画线优化"：让轨迹更平滑
│   ├── poly_traj/                  ← "画线工具2"：多项式轨迹
│   ├── traj_utils/                 ← "工具箱"：画图、显示等辅助功能
│   ├── lkh_mtsp_solver/            ← "算路线的专家"：LKH求解器
│   └── lkh_tsp_solver/             ← "算路线的专家2"：TSP求解器
└── uav_simulator/                  ← "模拟器"：模拟无人机飞行
    ├── map_generator/              ← 地图生成器
    ├── local_sensing/              ← 模拟摄像头
    ├── so3_quadrotor_simulator/    ← 模拟无人机物理运动
    ├── so3_control/                ← 模拟控制器
    └── ...
```

**类比**：
- `exploration_manager` = 总经理，决定整体策略
- `active_perception` = 侦察兵，负责发现新区域
- `plan_env` = 地图员，维护地图
- `path_searching` = 导航员，找路
- `bspline_opt` = 路线优化师，让路线更平滑
- `lkh_*_solver` = 数学专家，算最优路线

### 1.4 启动文件怎么看——swarm_exploration.launch

> **你要读的第一个文件**

这个文件是整个系统的"总开关"。它告诉你：

1. **启动多少架无人机**（默认 5 架）
2. **地图多大**（默认 35m × 35m × 3.5m）
3. **用哪张地图**（默认 `pillar.pcd`）
4. **为每架无人机启动哪些节点**

**怎么看**：打开 `RACER/swarm_exploration/exploration_manager/launch/swarm_exploration.launch`，你会看到类似这样的 XML 标签：

```xml
<arg name="drone_num" default="5"/>          <!-- 无人机数量 -->
<arg name="map_size_x" default="35.0"/>      <!-- 地图X方向尺寸 -->
<arg name="map_size_y" default="35.0"/>      <!-- 地图Y方向尺寸 -->
```

这些就是你可以调整的参数。

### 1.5 核心数据结构——expl_data.h（逐行讲解）

> **你要读的第二个文件**

路径：`RACER/swarm_exploration/exploration_manager/include/exploration_manager/expl_data.h`

这个文件定义了系统里最重要的数据结构。**你不需要一次看懂所有东西**，只需要知道：

#### DroneState：无人机的状态

```cpp
struct DroneState {
  Eigen::Vector3d pos_;              // 无人机的三维位置 (x, y, z)
  Eigen::Vector3d vel_;              // 无人机的三维速度
  double yaw_;                       // 无人机的朝向角（偏航角）
  double stamp_;                     // 时间戳（什么时候的数据）
  double recent_attempt_time_;       // 最近一次尝试分配的时间

  vector<int> grid_ids_;             // 这架无人机被分配到了哪些网格
  double recent_interact_time_;      // 最近一次和队友协商的时间
};
```

**类比**：这就是一架无人机的"身份证"——它在哪里、朝哪飞、负责哪些区域。

#### FSMData：有限状态机的数据

```cpp
struct FSMData {
  bool trigger_;                     // 是否被触发开始探索
  bool have_odom_;                   // 是否已经收到位置数据
  bool static_state_;                // 是否静止不动

  Eigen::Vector3d odom_pos_, odom_vel_;  // 当前的位置和速度
  Eigen::Quaterniond odom_orient_;        // 当前的姿态（用四元数表示）
  double odom_yaw_;                       // 当前的偏航角

  Eigen::Vector3d start_pt_, start_vel_;  // 起始状态
  vector<Eigen::Vector3d> start_poss;     // 起始位置列表
  bspline::Bspline newest_traj_;          // 最新的B样条轨迹

  // 集群避碰相关
  bool avoid_collision_;             // 是否需要避碰
  bool go_back_;                     // 是否需要返回
  ros::Time fsm_init_time_;          // 状态机初始化时间
  ros::Time last_check_frontier_time_; // 上次检查前沿的时间
};
```

**类比**：这就是无人机"大脑"的工作记忆——记住当前状态、位置、轨迹等信息。

#### ExplorationData：探索过程的数据

```cpp
struct ExplorationData {
  // 前沿相关
  vector<vector<Vector3d>> frontiers_;     // 所有前沿簇（每个簇是一组点）
  vector<vector<Vector3d>> dead_frontiers_; // 已经"死亡"的前沿（被探索完了）
  vector<pair<Vector3d, Vector3d>> frontier_boxes_; // 每个前沿的包围盒

  // 视点相关
  vector<Vector3d> points_;        // 每个前沿的最佳视点位置
  vector<double> yaws_;            // 每个最佳视点的偏航角
  vector<Vector3d> averages_;      // 每个前沿的平均位置
  vector<Vector3d> views_;         // 视点的可视化方向

  // 路径相关
  vector<Vector3d> frontier_tour_;  // 前沿访问路径
  vector<Vector3d> path_next_goal_; // 到下一个目标的路径
  vector<Vector3d> kino_path_;      // 运动学路径

  // 下一个目标
  Vector3d next_goal_;              // 下一个要去的位置
  Vector3d next_pos_;               // 下一个目标位置
  double next_yaw_;                 // 下一个目标朝向

  // 集群相关
  vector<DroneState> swarm_state_;  // 所有无人机的状态
  vector<double> pair_opt_stamps_;  // 分配请求的时间戳
  vector<double> pair_opt_res_stamps_; // 分配响应的时间戳
  vector<int> ego_ids_;             // 自己分配到的网格ID
  vector<int> other_ids_;           // 队友分配到的网格ID
  double pair_opt_stamp_;           // 当前分配请求的时间戳
  bool reallocated_;                // 是否刚刚重新分配过
  bool wait_response_;              // 是否在等待队友的响应

  // 网格相关
  vector<Vector3d> grid_tour_;      // 网格访问路径
  vector<int> last_grid_ids_;       // 上次分配的网格ID

  int plan_num_;                    // 规划次数
};
```

**类比**：这就是整个探索任务的"全局黑板"——所有无人机共享的信息都写在这里。

---

## 第二阶段：核心流程（3-5天）

> **目标**：理解一架无人机从"什么都不干"到"开始探索"再到"完成任务"的完整流程。

### 2.1 程序入口——exploration_node.cpp

> **你要读的第三个文件**

路径：`RACER/swarm_exploration/exploration_manager/src/exploration_node.cpp`

这是整个程序的"起点"。代码非常短，只有 22 行：

```cpp
#include <ros/ros.h>                                    // ROS的基本库
#include <exploration_manager/fast_exploration_fsm.h>   // 有限状态机的头文件

#include <plan_manage/backward.hpp>                     // 崩溃时打印调用栈（调试用）
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;  // 使用 fast_planner 命名空间

int main(int argc, char** argv) {
  // 1. 初始化ROS节点，节点名叫 "exploration_node"
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");  // 创建节点句柄，用于读取参数

  // 2. 创建有限状态机对象
  FastExplorationFSM expl_fsm;

  // 3. 初始化状态机（这里会初始化所有子模块）
  expl_fsm.init(nh);

  // 4. 等待1秒，让其他节点准备好
  ros::Duration(1.0).sleep();

  // 5. 进入ROS的事件循环，持续运行直到被关闭
  ros::spin();

  return 0;
}
```

**关键点**：
- `ros::init()` 告诉 ROS："我要启动一个叫 exploration_node 的程序"
- `expl_fsm.init(nh)` 是真正干活的地方——它初始化了地图、前沿提取、任务分配等所有模块
- `ros::spin()` 让程序一直运行，等待和处理各种事件（比如收到传感器数据、定时器触发等）

### 2.2 有限状态机（FSM）——无人机的"大脑开关"

> **要读的文件**：`fast_exploration_fsm.h` 和 `fast_exploration_fsm.cpp`

#### 状态定义

在 `fast_exploration_fsm.h` 第 36 行，定义了所有可能的状态：

```cpp
enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };
```

翻译成人话：

| 状态 | 英文 | 含义 |
|------|------|------|
| `INIT` | 初始化 | 等待无人机准备好（收到位置数据） |
| `WAIT_TRIGGER` | 等待触发 | 准备好了，等你在 Rviz 上点击"开始探索" |
| `PLAN_TRAJ` | 规划轨迹 | 计算下一段飞行路线 |
| `PUB_TRAJ` | 发布轨迹 | 把计算好的路线告诉控制器 |
| `EXEC_TRAJ` | 执行轨迹 | 无人机正在飞，等飞完或需要重新规划 |
| `IDLE` | 空闲 | 没有前沿了，闲着 |
| `FINISH` | 完成 | 探索任务完成 |

#### 状态转换图

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
                             ↓ (你在Rviz点击2D Nav Goal)
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
               │    │   执行轨迹       │  (replan: 需要重新规划)
               │    └────────┬────────┘
               │             ↓ (无前沿/完成)
               │    ┌─────────────────┐
               │    │      IDLE       │←── (100s无前沿)
               │    │   空闲状态       │──→ FINISH
               │    └─────────────────┘
               └─────────────────────────────
```

#### 关键回调函数

在 `fast_exploration_fsm.cpp` 里，有几个重要的函数：

1. **`triggerCallback()`**：当你在 Rviz 上点击"2D Nav Goal"时触发
   - 把状态从 `WAIT_TRIGGER` 切换到 `PLAN_TRAJ`

2. **`FSMCallback()`**：定时器回调，定期检查状态
   - 如果当前是 `PLAN_TRAJ`，就调用 `callExplorationPlanner()` 去规划
   - 如果规划成功，切换到 `PUB_TRAJ`，然后切换到 `EXEC_TRAJ`
   - 如果规划失败（没有前沿了），切换到 `IDLE`

3. **`optTimerCallback()`**：定时器回调，负责任务分配
   - 这是 Pair-wise 协商发生的地方
   - 当自己没有分配到网格时，会找一个队友发起协商

### 2.3 前沿提取——找到"未知的边界"（代码详解）

> **要读的文件**：`frontier_finder.cpp`

路径：`RACER/swarm_exploration/active_perception/src/frontier_finder.cpp`

#### 什么是前沿（Frontier）？

想象你在漆黑的房间里拿着手电筒。你照亮的区域是"已知的"，没照亮的是"未知的"。**已知和未知的交界处**就是"前沿"——你应该往那里走，因为那里有新的东西可以看。

#### 前沿提取的步骤（代码详解）

核心函数是 `searchFrontiers()`（第 55-141 行）：

```cpp
void FrontierFinder::searchFrontiers() {
  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();  // 清空临时前沿列表

  // 步骤1：获取地图更新区域
  // "地图刚刚更新了哪些部分？"
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, false);

  // 步骤2：清除已变化的前沿
  // "有些旧的前沿可能已经被探索完了，删掉"
  // 代码逻辑：遍历所有旧前沿，检查它们是否还满足前沿条件
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    if (haveAnyOverlap(iter->box_min_, iter->box_max_, mins, maxs) 
        && isFrontierChanged(*iter)) {
      // 如果前沿在更新区域内，且已经变了（不再是前沿了），就删掉
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else {
      ++rmv_idx;
      ++iter;
    }
  }

  // 步骤3：搜索新前沿种子
  // "在更新区域里找到'已知空闲且邻居有未知'的小格子"
  for (int z = min_id(2); z <= max_id(2); ++z)
    for (int x = min_id(0); x <= max_id(0); ++x)
      for (int y = min_id(1); y <= max_id(1); ++y) {
        Eigen::Vector3i cur(x, y, z);
        // 关键判断：当前格子是空闲的，且邻居有未知的
        if (frontier_flag_[toadr(cur)] == 0 
            && knownfree(cur) 
            && isNeighborUnknown(cur)) {
          // 找到种子了！从它开始扩展成一个完整的前沿簇
          expandFrontier(cur);
        }
      }

  // 步骤4：分裂过大前沿
  // "如果一个前沿太大了，拆成几个小的"
  splitLargeFrontiers(tmp_frontiers_);
}
```

#### 前沿判定条件（代码详解）

`isNeighborUnknown()` 函数（第 1077-1084 行）：

```cpp
inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  // 检查6个邻居（上下左右前后）
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    // 如果任何一个邻居是未知的，就返回true
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN) 
      return true;
  }
  return false;
}
```

**用人话说**：一个小格子被判定为"前沿种子"的条件是：
- 它自己是"已知空闲"的（FREE）
- 它的上下左右前后6个邻居里，至少有一个是"未知"的（UNKNOWN）

#### 区域生长（代码详解）

`expandFrontier()` 函数（第 143-182 行）：

```cpp
void FrontierFinder::expandFrontier(const Eigen::Vector3i& first) {
  // 用BFS（广度优先搜索）从种子开始扩展
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;

  Vector3d pos;
  edt_env_->sdf_map_->indexToPos(first, pos);
  expanded.push_back(pos);        // 把种子加入前沿
  cell_queue.push(first);         // 把种子加入队列
  frontier_flag_[toadr(first)] = 1;  // 标记为已处理

  // BFS扩展
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);  // 获取26个邻居
    for (auto nbr : nbrs) {
      int adr = toadr(nbr);
      // 如果邻居还没处理过，且在地图内，且是前沿种子
      if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) ||
          !(knownfree(nbr) && isNeighborUnknown(nbr)))
        continue;

      edt_env_->sdf_map_->indexToPos(nbr, pos);
      if (pos[2] < 0.2) continue;  // 去除地面附近的噪声
      expanded.push_back(pos);       // 加入前沿
      cell_queue.push(nbr);          // 加入队列继续扩展
      frontier_flag_[adr] = 1;       // 标记为已处理
    }
  }

  // 如果前沿太小（少于100个格子），就丢弃
  if (expanded.size() > cluster_min_) {
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);  // 计算前沿的平均位置、包围盒等
    tmp_frontiers_.push_back(frontier);
  }
}
```

**用人话说**：
1. 从一个种子格子开始
2. 看看它的26个邻居（上下左右前后+对角线）
3. 如果邻居也是"前沿种子"，就把它也加入这个前沿簇
4. 继续扩展，直到没有新的邻居可以加入
5. 如果最终的前沿太小（少于100个格子），就丢弃（可能是噪声）

---

## 第三阶段：任务分配（2-3天）⭐毕设重点

> **目标**：理解多架无人机怎么分工，这是你毕设最可能需要改造的部分。

### 3.1 分层网格——把地图切成"蛋糕"

> **要读的文件**：`hgrid.cpp`

路径：`RACER/swarm_exploration/active_perception/src/hgrid.cpp`

#### 为什么需要分网格？

5 架无人机在 35m × 35m 的区域里探索，如果每架都自己决定去哪里，很容易撞车或者重复探索。所以我们把地图切成若干个"格子"，每架无人机负责几个格子。

#### 分层网格（HGrid）的思路

```
Level 1（粗网格）：把地图切成大格子（比如 5m × 5m）
  ↓ 如果某个大格子里未知区域太多
Level 2（细网格）：把这个大格子再切成小格子
```

**类比**：
- 第一层：把一栋大楼分成 5 个区域（东区、西区、南区、北区、中区）
- 第二层：如果东区太大，再把东区分成"东1、东2、东3"

#### HGrid 构造函数（代码详解）

```cpp
HGrid::HGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh) {
  this->edt_ = edt;  // 保存环境指针（包含地图信息）

  // 读取参数
  nh.param("partitioning/consistent_cost", consistent_cost_, 3.5);  // 一致性代价
  nh.param("partitioning/use_swarm_tf", use_swarm_tf_, false);      // 是否使用集群坐标变换

  // 创建A*路径搜索器（用于计算两个点之间的路径代价）
  path_finder_.reset(new Astar);
  path_finder_->init(nh, edt);

  // 创建两层网格
  grid1_.reset(new UniformGrid(edt, nh, 1));  // Level 1：粗网格
  grid2_.reset(new UniformGrid(edt, nh, 2));  // Level 2：细网格

  // 初始化网格数据
  grid1_->initGridData();
  grid2_->initGridData();
}
```

#### 获取网格间的代价矩阵（代码详解）

`getCostMatrix()` 函数（第 277-338 行）：

```cpp
void HGrid::getCostMatrix(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
    const vector<vector<int>>& second_ids, const vector<int>& grid_ids, 
    Eigen::MatrixXd& mat) {
  
  const int drone_num = positions.size();  // 无人机数量
  const int grid_num = grid_ids.size();    // 网格数量
  const int dimen = 1 + drone_num + grid_num;  // 矩阵维度：1(虚拟仓库) + 无人机数 + 网格数

  // 初始化代价矩阵
  mat = Eigen::MatrixXd::Zero(dimen, dimen);

  // 虚拟仓库到无人机的代价（设为-1000，表示必须从无人机出发）
  for (int i = 0; i < drone_num; ++i) {
    mat(0, 1 + i) = -1000;   // 仓库 -> 无人机：-1000
    mat(1 + i, 0) = 1000;    // 无人机 -> 仓库：1000
  }

  // 虚拟仓库到网格的代价（设为1000，表示不能直接从仓库到网格）
  for (int i = 0; i < grid_num; ++i) {
    mat(0, 1 + drone_num + i) = 1000;
    mat(1 + drone_num + i, 0) = 0;
  }

  // 无人机之间的代价（设为10000，表示不应该直接从一架无人机飞到另一架）
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < drone_num; ++j) {
      mat(1 + i, 1 + j) = 10000;
    }
  }

  // 无人机到网格的代价（核心！）
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < grid_num; ++j) {
      // 计算从无人机当前位置到网格中心的代价
      double cost = getCostDroneToGrid(positions[i], grid_ids[j], first_ids[i]);
      mat(1 + i, 1 + drone_num + j) = cost;  // 无人机 -> 网格
      mat(1 + drone_num + j, 1 + i) = 0;      // 网格 -> 无人机：0（单向）
    }
  }

  // 网格之间的代价
  for (int i = 0; i < grid_num; ++i) {
    for (int j = i + 1; j < grid_num; ++j) {
      double cost = getCostGridToGrid(grid_ids[i], grid_ids[j], first_ids, second_ids, drone_num);
      mat(1 + drone_num + i, 1 + drone_num + j) = cost;
      mat(1 + drone_num + j, 1 + drone_num + i) = cost;  // 对称
    }
  }

  // 对角线设为1000（不能从自己到自己）
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 1000;
  }
}
```

**用人话说**：这个函数构建了一个"代价矩阵"，告诉求解器：
- 从每架无人机到每个网格要花多少时间
- 从每个网格到其他网格要花多少时间
- 然后求解器会根据这个矩阵，算出最优的分配方案

### 3.2 Pair-wise 分配——两两"商量"

> **要读的文件**：`fast_exploration_fsm.cpp` 里的 `optTimerCallback()` 函数

#### 怎么分配格子给无人机？

RACER 使用一种叫 **Pair-wise（配对）** 的分配方式：不是由一个"中央指挥官"统一分配，而是每两架无人机之间自己协商。

#### 流程

```
Drone A 想要探索                    Drone B 也在探索
      │                                 │
      │  1. A 发现自己没有分配到区域      │
      │  2. A 选一个最近的、很久没联系的无人机（比如 B）│
      │                                 │
      │──── 发送 PairOpt 消息 ─────────→│
      │     "我有这些格子，你有那些格子，  │
      │      我们一起算算怎么分最好"       │
      │                                 │
      │                    3. B 收到消息  │
      │                    4. B 检查时间戳│
      │                    5. B 接受或拒绝│
      │                                 │
      │←─── 返回 PairOptResponse ───────│
      │     "我同意/不同意这个分配方案"    │
      │                                 │
      │  7. A 验证响应，更新自己的分配     │
```

#### ACVRP 问题

在协商过程中，两架无人机会把各自负责的格子合并，然后用数学方法（ACVRP，带容量约束的车辆路径问题）求解：怎么分配这些格子，使得两架无人机飞行的总距离最短？

#### 震荡问题

这种方法有一个缺点——**震荡**。就是两架无人机可能反复"争抢"同一个格子，导致分配结果不断变化。震荡的原因：
1. 两架无人机同时发起请求
2. ACVRP 求解结果不稳定
3. 间隔时间太短，频繁协商

---

## 第四阶段：代价函数（1-2天）⭐毕设重点

> **目标**：理解系统怎么给"去哪个前沿"打分，这是你毕设另一个可能改造的部分。

### 4.1 什么是代价函数？——用"打分"来理解

**打个比方**：

你面前有 3 家餐厅，你要选一家去吃。你怎么选？

你会考虑：
1. **距离**：哪一家最近？（走路要多久）
2. **口味**：哪一家最好吃？
3. **排队**：哪一家不用等？

你会给每家餐厅打一个"综合分"，然后选分数最高的那家。

**代价函数就是这个"打分公式"**。只不过在这里，"餐厅"换成了"前沿"，"打分维度"换成了：
- 飞过去要多久（位置代价）
- 要不要转弯（速度方向代价）
- 要不要掉头（偏航代价）

**代价越低 = 越好**（和"分数越高越好"是反过来的）。

### 4.2 RACER 的代价函数——graph_node.cpp 逐行讲解

> **要读的文件**：`graph_node.cpp`

路径：`RACER/swarm_exploration/active_perception/src/graph_node.cpp`

#### computeCost() 函数（第 63-99 行）

```cpp
double ViewNode::computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1,
    const double& y2, const Vector3d& v1, const double& yd1, vector<Vector3d>& path) {
  // p1: 当前位置, p2: 目标位置
  // y1: 当前偏航角, y2: 目标偏航角
  // v1: 当前速度, yd1: 当前偏航角速度
  // path: 输出的路径

  // ========== 第1步：计算位置代价 ==========
  // "从p1飞到p2要多久？"
  double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_;
  // searchPath() 用A*算法找无碰撞路径，返回路径长度
  // vm_ 是最大速度（默认1.5 m/s）
  // 所以 pos_cost = 路径长度 / 最大速度 = 预计飞行时间

  // ========== 第2步：考虑速度方向代价 ==========
  // "如果当前有速度，考虑方向是否一致"
  if (v1.norm() > 1e-3) {  // 如果当前速度不为0
    Vector3d dir = (p2 - p1).normalized();   // 目标方向（从p1到p2的单位向量）
    Vector3d vdir = v1.normalized();          // 当前速度方向（单位向量）
    double diff = acos(vdir.dot(dir));        // 两个方向的夹角（弧度）
    // vdir.dot(dir) 是两个单位向量的点积，等于 cos(夹角)
    // acos() 是反余弦函数，得到夹角（弧度）
    
    pos_cost += w_dir_ * diff;  // 加上方向不一致的罚分
    // w_dir_ 是权重（默认1.5），夹角越大，罚分越重
    // 比如：当前往东飞，目标在北边，夹角90°，罚分 = 1.5 * π/2 ≈ 2.36
  }

  // ========== 第3步：计算偏航代价 ==========
  // "当前朝向和目标朝向的差值"
  double diff = fabs(y2 - y1);           // 偏航角差值的绝对值
  diff = min(diff, 2 * M_PI - diff);     // 取较短的旋转方向
  // 比如：当前朝0°，目标朝350°，差10°比差350°更短
  double yaw_cost = diff / yd_;           // 偏航代价 = 角度差 / 最大偏航速率
  // yd_ 是最大偏航速率（默认80°/s = 1.396 rad/s）

  // ========== 第4步：返回总代价 ==========
  return max(pos_cost, yaw_cost);
  // 取位置代价和偏航代价的最大值作为总代价
  // 为什么取最大值？因为飞行和转向是同时进行的，总时间取决于较慢的那个
}
```

#### searchPath() 函数（第 32-61 行）

```cpp
double ViewNode::searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path) {
  // 首先尝试直线连接（最快）
  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);  // 从p1到p2做射线检测
  while (caster_->nextId(idx)) {
    // 如果射线碰到障碍物或出界，就不安全
    if (map_->getInflateOccupancy(idx) == 1 || !map_->isInBox(idx)) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = { p1, p2 };  // 直线安全，直接返回两点
    return (p1 - p2).norm();  // 返回直线距离
  }

  // 直线不安全，用A*搜索
  vector<double> res = { 0.4 };  // 搜索分辨率
  for (int k = 0; k < res.size(); ++k) {
    astar_->reset();
    astar_->setResolution(res[k]);
    if (astar_->search(p1, p2) == Astar::REACH_END) {
      path = astar_->getPath();  // 获取A*找到的路径
      return astar_->pathLength(path);  // 返回路径长度
    }
  }

  // A*也找不到路径，返回一个很大的代价
  path = { p1, p2 };
  return 100;  // 表示这条路很难走
}
```

**用人话说**：
1. 先试试能不能直线飞过去（最快）
2. 如果不行（有障碍物），用A*算法找一条绕路的路径
3. 如果A*也找不到，就给一个很高的代价（表示这条路很差）

#### 代价函数总结

| 代价项 | 公式 | 含义 | 例子 |
|--------|------|------|------|
| `pos_cost` | `路径长度 / vm_` | 飞过去要多久 | 路径10m，速度1.5m/s，代价=6.67s |
| `w_dir_ * diff` | `1.5 * 夹角` | 要不要急转弯 | 当前往东飞，目标在北边，夹角90°，罚分≈2.36 |
| `yaw_cost` | `角度差 / yd_` | 要不要掉头 | 当前朝东(0°)，目标朝西(180°)，偏航代价≈2.26s |
| `max(pos, yaw)` | `max(位置代价, 偏航代价)` | 取较慢的那个 | 飞行6.67s，转向2.26s，总代价=6.67s |

---

## 第五阶段：轨迹规划（2-3天）

> **目标**：理解系统怎么把"去某个前沿"的决策变成一条可执行的飞行轨迹。

### 5.1 探索管理器如何规划——fast_exploration_manager.cpp 详解

> **要读的文件**：`fast_exploration_manager.cpp`

路径：`RACER/swarm_exploration/exploration_manager/src/fast_exploration_manager.cpp`

#### planExploreMotion() 函数（第 120-297 行）

这是整个探索规划的"总调度"：

```cpp
int FastExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  // pos: 当前位置, vel: 当前速度, acc: 当前加速度, yaw: 当前偏航角

  // 步骤1：找到网格级和前沿级的路径
  vector<int> grid_ids, frontier_ids;
  findGridAndFrontierPath(pos, vel, yaw, grid_ids, frontier_ids);

  // 步骤2：根据结果决定下一步
  if (grid_ids.empty()) {
    // 没有分配到网格，找最近的目标飞过去
    return NO_GRID;
  } 
  else if (frontier_ids.size() == 0) {
    // 分配的网格里没有前沿，飞到网格中心
    Eigen::Vector3d grid_center = ed_->grid_tour_[1];
    // 找离网格中心最近的前沿
    double min_cost = 100000;
    int min_cost_id = -1;
    for (int i = 0; i < ed_->averages_.size(); ++i) {
      vector<Eigen::Vector3d> path;
      double cost = ViewNode::computeCost(
          grid_center, ed_->averages_[i], 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
      if (cost < min_cost) {
        min_cost = cost;
        min_cost_id = i;
      }
    }
    next_pos = ed_->points_[min_cost_id];
    next_yaw = ed_->yaws_[min_cost_id];
  } 
  else if (frontier_ids.size() == 1) {
    // 只有一个前沿，直接去
    // ... 省略具体代码 ...
  } 
  else {
    // 多个前沿，做局部优化
    // ... 省略具体代码 ...
  }

  // 步骤3：规划轨迹到下一个目标
  if (planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw) == FAIL) {
    return FAIL;
  }

  return SUCCEED;
}
```

#### findGridAndFrontierPath() 函数（第 417-464 行）

```cpp
void FastExplorationManager::findGridAndFrontierPath(const Vector3d& cur_pos,
    const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids,
    vector<int>& frontier_ids) {
  
  // 步骤1：网格级规划（决定去哪个区域）
  vector<int> ego_ids;
  vector<vector<int>> other_ids;
  if (!findGlobalTourOfGrid(positions, velocities, ego_ids, other_ids)) {
    grid_ids = {};  // 没有分配到网格
    return;
  }
  grid_ids = ego_ids;  // 保存分配到的网格ID

  // 步骤2：前沿级规划（决定去哪个前沿）
  vector<int> ftr_ids;
  hgrid_->getFrontiersInGrid(ego_ids, ftr_ids);  // 获取当前网格内的前沿

  if (ftr_ids.empty()) {
    frontier_ids = {};  // 网格内没有前沿
    return;
  }

  // 考虑下一个网格（让路径更连贯）
  Eigen::Vector3d grid_pos;
  double grid_yaw;
  vector<Eigen::Vector3d> grid_pos_vec;
  if (hgrid_->getNextGrid(ego_ids, grid_pos, grid_yaw)) {
    grid_pos_vec = { grid_pos };
  }

  // 求解前沿TSP（旅行商问题）
  findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr_ids, grid_pos_vec, frontier_ids);
}
```

#### refineLocalTour() 函数（第 583-656 行）

局部视点优化：

```cpp
void FastExplorationManager::refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<vector<Vector3d>>& n_points,
    const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts,
    vector<double>& refined_yaws) {
  
  // 创建图搜索问题
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // 添加当前状态作为起点
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);

  // 为每个前沿的每个候选视点创建节点
  for (int i = 0; i < n_points.size(); ++i) {
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // 连接当前组的所有节点到这个节点
      for (auto nd : last_group) g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);
    }
    last_group = cur_group;
    cur_group.clear();
  }

  // 用Dijkstra算法搜索最优视点序列
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  // 返回搜索到的最优序列
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }
}
```

**用人话说**：
1. 为每个前沿的每个候选视点创建一个"节点"
2. 用Dijkstra算法找一条"总代价最小"的路径，经过每个前沿至少一次
3. 返回这条路径上的第一个视点作为下一个目标

---

## 附录A：常见术语中英文对照

| 英文 | 中文 | 一句话解释 |
|------|------|-----------|
| Frontier | 前沿 | 已知区域和未知区域的交界处 |
| Viewpoint | 视点 | 无人机应该飞到的位置和朝向 |
| HGrid | 分层网格 | 把地图分成多层格子来管理 |
| Pair-wise | 配对 | 两架无人机之间协商分配 |
| ACVRP | 带容量约束的车辆路径问题 | 一种数学优化问题 |
| TSP | 旅行商问题 | 找最短路线的经典问题 |
| B-Spline | B样条曲线 | 把控制点变成平滑曲线的工具 |
| SDF | 符号距离场 | 每个点到最近障碍物的距离 |
| FSM | 有限状态机 | 用几个状态管理程序流程 |
| A* | A-star算法 | 经典的最短路径搜索算法 |
| DQN | 深度Q网络 | 一种强化学习算法（你可能要用的） |
| ROS | 机器人操作系统 | 机器人模块间的通信框架 |
| LKH | 一种TSP近似求解算法 | 很快地找到接近最优的路线 |
| Jerk | 加加速度 | 加速度的变化率，越小飞行越平滑 |
| Yaw | 偏航角 | 无人机机头的朝向（左右旋转） |
| Ray Casting | 射线投射 | 从一个点发射射线，检测碰撞 |
| BFS | 广度优先搜索 | 一种图搜索算法，用于前沿扩展 |
| Dijkstra | 迪杰斯特拉算法 | 一种最短路径算法，用于视点选择 |

---

## 附录B：你的毕设改造路线图

假设你的毕设方向是"用深度强化学习优化无人机集群探索"，以下是建议的改造路线：

### 第一步：跑通现有系统（1周）

1. 按照项目 README 编译和运行
2. 在 Rviz 里触发探索，观察 5 架无人机如何协同工作
3. 理解各个 ROS 话题的数据流

### 第二步：深入理解核心代码（2周）

按照本文档的阅读顺序，重点精读：
- `exploration_node.cpp`：理解程序入口
- `fast_exploration_fsm.cpp`：理解状态转换
- `fast_exploration_manager.cpp`：理解探索管理
- `frontier_finder.cpp`：理解前沿提取
- `hgrid.cpp`：理解任务分配
- `graph_node.cpp`：理解代价函数

### 第三步：设计你的改进方案（1周）

1. 确定你要替换哪个模块（分配 or 代价函数 or 两者都换）
2. 设计你的算法（DQN 的状态、动作、奖励）
3. 画出修改方案的流程图

### 第四步：实现和测试（3-4周）

1. 在现有代码框架里插入你的算法
2. 先用少量无人机（2-3架）测试
3. 逐步增加到 5 架
4. 对比你改进前后的探索效率

### 第五步：撰写论文（2周）

1. 介绍 RACER 的基本框架
2. 说明你改进了什么、为什么这样改进
3. 展示实验结果对比

---

**最后的建议**：

1. **不要试图一次看懂所有代码**——先跑通系统，再逐步深入
2. **多画图**——把数据流画出来，比看代码有效 10 倍
3. **多改参数试试**——改改无人机数量、地图大小，观察系统行为变化
4. **善用 ROS 工具**——`rostopic echo` 可以查看话题数据，`rqt_graph` 可以看节点关系
5. **遇到不懂的术语**——回来查本文档的附录 A

祝你的毕设顺利！🎓
