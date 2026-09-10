# RACER 毕设入门讲解文档（代码详解版）

> **写给零基础的你**：本文假设你只有最基本的计算机知识，对无人机、集群算法、机器人操作系统几乎不了解。每个概念都会结合**真实源码**讲解，告诉你"代码在哪里"、"这段代码在做什么"。

---

## 目录

- [第零章：预备知识](#第零章预备知识你需要知道的基本概念)
- [第一阶段：整体理解（1-2天）](#第一阶段整体理解1-2天)
- [第二阶段：核心流程（3-5天）](#第二阶段核心流程3-5天)
- [第三阶段：任务分配（2-3天）⭐毕设重点](#第三阶段任务分配2-3天毕设重点)
- [第四阶段：代价函数（1-2天）⭐毕设重点](#第四阶段代价函数1-2天毕设重点)
- [第五阶段：轨迹规划（2-3天）](#第五阶段轨迹规划2-3天)
- [附录A：术语对照表](#附录a常见术语中英文对照)
- [附录B：毕设改造路线图](#附录b你的毕设改造路线图)

---

## 第零章：预备知识——你需要知道的基本概念

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

### 1.2 项目的"骨架"——目录结构

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

### 1.3 启动文件怎么看——swarm_exploration.launch

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

### 1.4 核心数据结构——expl_data.h（⭐必须精读）

> **你要读的第二个文件**
>
> 路径：`RACER/swarm_exploration/exploration_manager/include/exploration_manager/expl_data.h`

这个文件定义了系统里**最重要的数据结构**。就像游戏的"角色属性面板"，每个结构体描述了一类对象的状态。

#### DroneState——无人机的"身份证"

```cpp
struct DroneState {
  Eigen::Vector3d pos_;        // 位置 (x, y, z)
  Eigen::Vector3d vel_;        // 速度
  double yaw_;                 // 偏航角（机头朝向）
  double stamp_;               // 时间戳（什么时候的数据）
  double recent_attempt_time_; // 最近一次尝试分配的时间

  vector<int> grid_ids_;       // 这架无人机负责哪些网格【关键！】
  double recent_interact_time_;// 最近一次和队友交互的时间
};
```

**用人话说**：每架无人机都维护一个 `DroneState`，记录自己的位置、速度、朝向、以及"我负责哪些区域"（`grid_ids_`）。队友之间通过广播这个结构体来互相了解状态。

#### ExplorationData——探索过程的"全局黑板"

```cpp
struct ExplorationData {
  // 前沿相关
  vector<vector<Vector3d>> frontiers_;     // 所有前沿簇（每个簇是一组点）
  vector<pair<Vector3d, Vector3d>> frontier_boxes_; // 每个前沿的包围盒
  vector<Vector3d> points_;               // 每个前沿的最优视点位置
  vector<double> yaws_;                   // 每个前沿的最优视点朝向
  vector<Vector3d> averages_;             // 每个前沿的平均位置

  // 路径相关
  vector<int> refined_ids_;               // 被选中的前沿ID
  vector<Vector3d> refined_points_;       // 精炼后的视点位置
  Vector3d next_pos_;                     // 下一个要飞去的位置【核心输出！】
  double next_yaw_;                       // 下一个要朝的方向【核心输出！】

  // 集群相关
  vector<DroneState> swarm_state_;        // 所有无人机的状态【集群协调的基础】
  vector<int> ego_ids_, other_ids_;       // Pair-wise分配结果

  // 网格相关
  vector<Vector3d> grid_tour_;            // 网格级巡游路径
  vector<int> last_grid_ids_;             // 上一轮的网格分配
};
```

**用人话说**：`ExplorationData` 是一个"全局黑板"，整个探索过程的所有中间结果都写在这里。每次规划时，系统从这里读取数据、计算、再把结果写回去。

> **你现在不需要记住每个字段**，只需要知道：如果以后你要加新的数据（比如 DQN 的输出），大概率要在这里加字段。

---

## 第二阶段：核心流程（3-5天）

> **目标**：理解一架无人机从"什么都不干"到"开始探索"再到"完成任务"的完整流程。

### 2.1 程序入口——exploration_node.cpp（最短的文件）

> 路径：`RACER/swarm_exploration/exploration_manager/src/exploration_node.cpp`

这是整个探索节点的 `main()` 函数，只有 22 行：

```cpp
int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");  // 初始化ROS节点
  ros::NodeHandle nh("~");                    // 创建节点句柄

  FastExplorationFSM expl_fsm;               // 创建状态机对象
  expl_fsm.init(nh);                         // 初始化（注册所有回调函数）

  ros::Duration(1.0).sleep();                // 等待1秒让系统稳定
  ros::spin();                               // 进入主循环，等待消息
  return 0;
}
```

**用人话说**：这个文件做的事情就是"创建一个状态机，初始化它，然后开始运行"。所有的逻辑都在 `FastExplorationFSM` 类里。

### 2.2 有限状态机（FSM）——无人机的"大脑开关"

> 路径：`fast_exploration_fsm.h` 和 `fast_exploration_fsm.cpp`

#### 状态定义

在头文件中，状态用枚举类型定义：

```cpp
enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };
```

| 状态 | 含义 | 打个比方 |
|------|------|----------|
| `INIT` | 初始化，等待里程计就绪 | 你刚醒来，还在穿鞋 |
| `WAIT_TRIGGER` | 等待你在Rviz上点击"开始" | 穿好鞋了，等发令枪响 |
| `PLAN_TRAJ` | 计算下一段飞行轨迹 | 在地图上画路线 |
| `PUB_TRAJ` | 把轨迹发布出去 | 把路线告诉司机 |
| `EXEC_TRAJ` | 无人机正在飞 | 司机正在开车 |
| `IDLE` | 没有前沿了，空闲等待 | 到目的地了，在休息 |
| `FINISH` | 探索完成 | 任务结束，下班 |

#### 状态转换——FSMCallback

```cpp
void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  switch (state_) {
    case INIT: {
      // 等待里程计（odometry）数据就绪
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom");
        return;  // 还没收到位置数据，继续等
      }
      if ((ros::Time::now() - fd_->fsm_init_time_).toSec() < 2.0) {
        ROS_WARN_THROTTLE(1.0, "wait for init");
        return;  // 初始化时间不够，继续等
      }
      transitState(WAIT_TRIGGER, "FSM");  // 跳转到等待触发状态
      break;
    }

    case WAIT_TRIGGER: {
      // 什么都不做，等你在Rviz上点击"开始探索"
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case PLAN_TRAJ: {
      // 【核心】规划探索轨迹
      // 如果是悬停状态（第一次规划），从当前位置开始
      if (fd_->static_state_) {
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();
        fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
      } else {
        // 如果正在飞行中（重新规划），从"replan_time秒后"的位置开始
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        // ... 类似地获取加速度和偏航
      }

      // 调用探索规划器
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");    // 规划成功，去发布轨迹
      } else if (res == NO_GRID) {
        transitState(IDLE, "FSM");        // 没有网格了，空闲
      }
      break;
    }

    case PUB_TRAJ: {
      // 把规划好的B-Spline轨迹发布出去
      bspline_pub_.publish(fd_->newest_traj_);    // 发布给轨迹服务器
      swarm_traj_pub_.publish(fd_->newest_traj_); // 发布给队友（避碰用）
      transitState(EXEC_TRAJ, "FSM");             // 跳转到执行状态
      break;
    }

    case EXEC_TRAJ: {
      // 检查是否需要重新规划
      bool need_replan = false;
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      if (expl_manager_->frontier_finder_->isFrontierCovered()) {
        need_replan = true;  // 目标前沿已经被探索完了
      } else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
        need_replan = true;  // 轨迹快飞完了
      } else if (t_cur > fp_->replan_thresh3_) {
        need_replan = true;  // 定时重新规划
      }

      if (need_replan) {
        if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
          transitState(PLAN_TRAJ, "FSM");  // 有新前沿，重新规划
        } else {
          transitState(IDLE, "FSM");       // 没有前沿了
        }
      }
      break;
    }
  }
}
```

**用人话说**：

整个状态机的工作流程就是：

```
你点击"开始" → 规划轨迹 → 发布轨迹 → 无人机飞 → 飞完/需要更新 → 重新规划 → ...
                                                        ↓ 没有前沿了
                                                      空闲/完成
```

#### 触发回调——triggerCallback

```cpp
void FastExplorationFSM::triggerCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  if (state_ != WAIT_TRIGGER) return;  // 只在等待状态才响应
  fd_->trigger_ = true;
  fd_->start_pos_ = fd_->odom_pos_;  // 记录起始位置

  // 更新前沿结构，如果有前沿就进入规划状态
  if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
    transitState(PLAN_TRAJ, "triggerCallback");
  } else
    transitState(FINISH, "triggerCallback");  // 没有前沿，直接完成
}
```

**用人话说**：你在 Rviz 上点击"2D Nav Goal"时，这个函数被调用。它做的事情是"收到出发指令，开始规划"。

### 2.3 前沿提取——找到"未知的边界"

> 路径：`RACER/swarm_exploration/active_perception/src/frontier_finder.cpp`

#### 前沿的数据结构

在头文件 `frontier_finder.h` 中定义：

```cpp
// 一个"视点"——无人机应该飞到的位置和朝向
struct Viewpoint {
  Vector3d pos_;       // 位置 (x, y, z)
  double yaw_;         // 朝向（偏航角）
  int visib_num_;      // 从这个位置能看到多少个前沿体素
};

// 一个"前沿簇"——一组相邻的未知边界体素
struct Frontier {
  vector<Vector3d> cells_;          // 属于这个前沿的所有体素位置
  Vector3d average_;                // 前沿的平均位置
  int id_;                          // 前沿的编号
  vector<Viewpoint> viewpoints_;    // 能覆盖这个前沿的候选视点
  Vector3d box_min_, box_max_;     // 前沿的包围盒
  list<vector<Vector3d>> paths_;   // 到其他前沿的路径
  list<double> costs_;             // 到其他前沿的代价
};
```

**用人话说**：
- `Frontier` = "地图上的一个未探索区域"（已知和未知的交界处）
- `Viewpoint` = "站在哪里看这个区域最合适"

#### 核心搜索函数——searchFrontiers

```cpp
void FrontierFinder::searchFrontiers() {
  tmp_frontiers_.clear();

  // 第1步：获取地图更新区域
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, false);

  // 第2步：清除已经变化的旧前沿
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    if (haveAnyOverlap(iter->box_min_, iter->box_max_, mins, maxs)
        && isFrontierChanged(*iter)) {
      // 这个前沿的地图已经更新了，需要重新搜索
      resetFlag(iter, frontiers_);
    } else {
      ++iter;
    }
  }

  // 第3步：在更新区域内搜索新的前沿种子
  for (int z = min_id(2); z <= max_id(2); ++z)
    for (int x = min_id(0); x <= max_id(0); ++x)
      for (int y = min_id(1); y <= max_id(1); ++y) {
        Eigen::Vector3i cur(x, y, z);
        // 关键条件：体素未被标记 && 是已知空闲 && 邻居有未知
        if (frontier_flag_[toadr(cur)] == 0
            && knownfree(cur)
            && isNeighborUnknown(cur)) {
          expandFrontier(cur);  // 从种子扩展成完整的前沿簇
        }
      }

  // 第4步：分裂过大的前沿
  splitLargeFrontiers(tmp_frontiers_);
}
```

#### 前沿判定条件——isNeighborUnknown

```cpp
bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  auto nbrs = tenNeighbors(voxel);  // 检查10个邻居
  for (auto nbr : nbrs) {
    if (!inmap(nbr)) continue;
    // 如果邻居是"未知"的，说明这是一个前沿
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
      return true;
  }
  return false;
}
```

**用人话说**：一个小格子被判定为"前沿种子"的条件是：
1. 它自己是"已知空闲"的（FREE）——无人机能飞到这里
2. 它的邻居里至少有一个是"未知"的（UNKNOWN）——旁边还有没看过的地方

#### 区域生长——expandFrontier

```cpp
void FrontierFinder::expandFrontier(const Eigen::Vector3i& first) {
  queue<Eigen::Vector3i> cell_queue;  // BFS队列
  vector<Eigen::Vector3d> expanded;

  Vector3d pos;
  edt_env_->sdf_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  frontier_flag_[toadr(first)] = 1;  // 标记为已处理

  // BFS扩展：把相邻的同类体素合并成一个前沿簇
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);  // 检查所有26个邻居
    for (auto nbr : nbrs) {
      int adr = toadr(nbr);
      if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr)
          || !(knownfree(nbr) && isNeighborUnknown(nbr)))
        continue;  // 跳过已处理的、不在地图里的、不符合条件的

      edt_env_->sdf_map_->indexToPos(nbr, pos);
      if (pos[2] < 0.2) continue;  // 忽略地面附近的噪声
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }

  // 只有体素数量超过阈值才保留（过滤噪声）
  if (expanded.size() > cluster_min_) {
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);  // 计算包围盒、视点等
    tmp_frontiers_.push_back(frontier);
  }
}
```

**用人话说**：找到一个种子后，用 BFS（广度优先搜索）把周围所有符合条件的格子都"圈"进来，形成一个完整的前沿簇。就像用油漆桶填充——从一个点开始，向四周扩散。

---

## 第三阶段：任务分配（2-3天）⭐毕设重点

> **目标**：理解多架无人机怎么分工，这是你毕设最可能需要改造的部分。

### 3.1 分层网格——把地图切成"蛋糕"

> 路径：`RACER/swarm_exploration/active_perception/src/hgrid.cpp`

#### 为什么需要分网格？

5 架无人机在 35m × 35m 的区域里探索，如果每架都自己决定去哪里，很容易撞车或者重复探索。所以我们把地图切成若干个"格子"，每架无人机负责几个格子。

#### 分层网格（HGrid）的构造

```cpp
HGrid::HGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh) {
  this->edt_ = edt;

  // 读取参数
  nh.param("partitioning/consistent_cost", consistent_cost_, 3.5);

  // 创建两层网格
  grid1_.reset(new UniformGrid(edt, nh, 1));  // Level 1: 粗网格
  grid2_.reset(new UniformGrid(edt, nh, 2));  // Level 2: 细网格

  // 初始化网格数据
  grid1_->initGridData();
  grid2_->initGridData();
}
```

**类比**：
- 第一层：把一栋大楼分成 5 个区域（东区、西区、南区、北区、中区）
- 第二层：如果东区太大，再把东区分成"东1、东2、东3"

#### 网格状态更新——updateGridData

```cpp
void HGrid::updateGridData(const int& drone_id, vector<int>& grid_ids,
    bool reallocated, const vector<int>& last_grid_ids,
    vector<int>& first_ids, vector<int>& second_ids) {

  // 把统一的grid_id拆分成两层的id
  vector<int> grid_ids1, grid_ids2;
  const int grid_num1 = grid1_->grid_data_.size();
  for (auto id : grid_ids) {
    if (id < grid_num1)
      grid_ids1.push_back(id);            // 属于Level 1
    else
      grid_ids2.push_back(id - grid_num1); // 属于Level 2
  }

  // 在Level 1更新
  vector<int> parti_ids1, parti_ids1_all;
  grid1_->updateGridData(drone_id, grid_ids1, parti_ids1, parti_ids1_all);

  // 如果Level 1的某个网格需要细分，转换成Level 2的id
  vector<int> fine_ids;
  for (auto id : parti_ids1) {
    vector<int> tmp_ids;
    coarseToFineId(id, tmp_ids);  // 粗网格ID → 细网格ID列表
    fine_ids.insert(fine_ids.end(), tmp_ids.begin(), tmp_ids.end());
  }
  grid_ids2.insert(grid_ids2.end(), fine_ids.begin(), fine_ids.end());

  // 在Level 2更新
  grid2_->updateGridData(drone_id, grid_ids2, parti_ids2, parti_ids2_all);

  // 合并两层的id
  grid_ids = grid_ids1;
  for (auto& id : grid_ids2) {
    grid_ids.push_back(id + grid_num1);  // 加上偏移量
  }

  // 维护路径一致性
  getConsistentGrid(last_grid_ids, grid_ids, first_ids, second_ids);
}
```

**用人话说**：每次更新时，系统先看"我的网格分配"，然后在两层网格上分别更新。如果某个粗网格太大了，就把它拆成几个细网格。

### 3.2 Pair-wise 分配——两两"商量"

> 路径：`fast_exploration_fsm.cpp` 第715行 `optTimerCallback()`

这是集群协调的**核心机制**。不是由一个"中央指挥官"统一分配，而是每两架无人机之间自己协商。

#### 选择协商对象

```cpp
void FastExplorationFSM::optTimerCallback(const ros::TimerEvent& e) {
  if (state_ == INIT) return;

  auto& states = expl_manager_->ed_->swarm_state_;
  auto& state1 = states[getId() - 1];  // 自己的状态
  bool urgent = state1.grid_ids_.empty();  // 是否紧急（没有分配到网格）

  // 避免过于频繁的尝试
  auto tn = ros::Time::now().toSec();
  if (tn - state1.recent_attempt_time_ < fp_->attempt_interval_) return;

  // 选择一个队友来协商
  int select_id = -1;
  double max_interval = -1.0;
  for (int i = 0; i < states.size(); ++i) {
    if (i + 1 <= getId()) continue;  // 避免重复选择

    // 过滤条件：
    if (tn - states[i].stamp_ > 0.2) continue;              // 对方最近有消息
    if (tn - states[i].recent_attempt_time_ < fp_->attempt_interval_) continue;
    if (tn - states[i].recent_interact_time_ < fp_->pair_opt_interval_) continue;
    if (states[i].grid_ids_.size() + state1.grid_ids_.size() == 0) continue;

    // 选择"最久没交互"的那个队友
    double interval = tn - states[i].recent_interact_time_;
    if (interval <= max_interval) continue;
    select_id = i + 1;
    max_interval = interval;
  }
  if (select_id == -1) return;  // 没有合适的队友
```

**用人话说**：每隔 0.05 秒检查一次——"我有没有分配到网格？如果没有，找一个最近没联系过的队友来商量。"

#### 合并网格并求解

```cpp
  // 合并双方的网格
  unordered_map<int, char> opt_ids_map;
  auto& state2 = states[select_id - 1];
  for (auto id : state1.grid_ids_) opt_ids_map[id] = 1;  // 自己的网格
  for (auto id : state2.grid_ids_) opt_ids_map[id] = 1;  // 对方的网格
  vector<int> opt_ids;
  for (auto pair : opt_ids_map) opt_ids.push_back(pair.first);

  // 也把"没有被分配"的活跃网格加进来
  vector<int> actives, missed;
  expl_manager_->hgrid_->getActiveGrids(actives);
  findUnallocated(actives, missed);  // 找到没人负责的网格
  opt_ids.insert(opt_ids.end(), missed.begin(), missed.end());

  // 调用分配算法（ACVRP求解）
  vector<Eigen::Vector3d> positions = { state1.pos_, state2.pos_ };
  vector<int> ego_ids, other_ids;
  expl_manager_->allocateGrids(positions, velocities,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 },
      opt_ids, ego_ids, other_ids);

  // ego_ids: 给自己分配的网格
  // other_ids: 给对方分配的网格
```

**用人话说**：把两个人的"地盘"合在一起，用数学方法重新分，看怎么分两个人飞行的总距离最短。

#### 发送分配请求

```cpp
  // 构造PairOpt消息
  exploration_manager::PairOpt opt;
  opt.from_drone_id = getId();       // 谁发的
  opt.to_drone_id = select_id;       // 发给谁
  opt.stamp = tn;                    // 时间戳（防重复）
  for (auto id : ego_ids) opt.ego_ids.push_back(id);    // 建议自己拿这些
  for (auto id : other_ids) opt.other_ids.push_back(id); // 建议对方拿这些

  // 连发10次（防止丢消息）
  for (int i = 0; i < fp_->repeat_send_num_; ++i)
    opt_pub_.publish(opt);

  // 等待对方确认
  ed->ego_ids_ = ego_ids;
  ed->other_ids_ = other_ids;
  ed->wait_response_ = true;
```

#### 接收并处理请求——optMsgCallback

```cpp
void FastExplorationFSM::optMsgCallback(
    const exploration_manager::PairOptConstPtr& msg) {
  if (msg->from_drone_id == getId()) return;  // 忽略自己发的
  if (msg->to_drone_id != getId()) return;    // 不是发给我的

  // 检查时间戳，避免处理过期消息
  if (msg->stamp <= expl_manager_->ed_->pair_opt_stamps_[
      msg->from_drone_id - 1] + 1e-4)
    return;

  // 检查是否太频繁
  if (msg->stamp - state2.recent_attempt_time_ < fp_->attempt_interval_) {
    ROS_WARN("Reject frequent attempt");
    response.status = 2;  // 拒绝：太频繁了
    return;
  }

  // 接受分配结果
  response.status = 1;  // 接受
  // ... 更新自己的grid_ids
```

**用人话说**：收到队友的分配请求后，检查"是不是太频繁了"、"时间戳对不对"，然后决定接受还是拒绝。如果接受，就更新自己的网格分配。

### 3.3 ACVRP 求解——怎么分配最优

> 路径：`fast_exploration_manager.cpp` 第658行 `allocateGrids()`

```cpp
void FastExplorationManager::allocateGrids(
    const vector<Eigen::Vector3d>& positions,     // 两架无人机的位置
    const vector<Eigen::Vector3d>& velocities,    // 速度
    const vector<vector<int>>& first_ids,         // 上一轮的第一个网格
    const vector<vector<int>>& second_ids,        // 上一轮的第二个网格
    const vector<int>& grid_ids,                  // 要分配的网格ID列表
    vector<int>& ego_ids,                         // 输出：给自己的
    vector<int>& other_ids) {                     // 输出：给对方的

  // 特殊情况：只有1个网格，直接比距离
  if (grid_ids.size() == 1) {
    auto pt = hgrid_->getCenter(grid_ids.front());
    vector<Eigen::Vector3d> path;
    double d1 = ViewNode::computeCost(positions[0], pt, 0, 0,
        Eigen::Vector3d(0, 0, 0), 0, path);
    double d2 = ViewNode::computeCost(positions[1], pt, 0, 0,
        Eigen::Vector3d(0, 0, 0), 0, path);
    if (d1 < d2) {
      ego_ids = grid_ids;    // 自己更近，归自己
      other_ids = {};
    } else {
      ego_ids = {};
      other_ids = grid_ids;  // 对方更近，归对方
    }
    return;
  }

  // 正常情况：多个网格，构建ACVRP问题
  Eigen::MatrixXd mat;
  hgrid_->getCostMatrix(positions, velocities, first_ids,
      second_ids, grid_ids, mat);

  // 计算容量约束（基于未知体素数量）
  int capacity = 0;
  for (int i = 0; i < grid_ids.size(); ++i) {
    int unum = hgrid_->getUnknownCellsNum(grid_ids[i]);
    capacity += unum;
  }
  capacity = capacity * 0.75 * 0.1;  // 容量 = 总未知数 × 0.75 × 0.1

  // 构造问题文件，调用LKH求解器
  ofstream file(ep_->mtsp_dir_ + "/amtsp3_" +
      to_string(ep_->drone_id_) + ".atsp");
  file << "TYPE : ACVRP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "CAPACITY : " + to_string(capacity) + "\n";
  file << "VEHICLES : " + to_string(drone_num) + "\n";
  // ... 写入代价矩阵 ...

  // 调用LKH求解器
  lkh_mtsp_solver::SolveMTSP srv;
  acvrp_client_.call(srv);
  // ... 读取结果，分配给ego_ids和other_ids ...
}
```

**用人话说**：
1. 把两架无人机的位置和所有待分配网格作为节点
2. 计算节点间的距离矩阵（代价矩阵）
3. 加上"容量约束"（每个网格的未知区域大小就是它的"重量"）
4. 用 LKH 求解器求解 ACVRP 问题
5. 得到最优分配方案

### 3.4 消息定义——无人机之间怎么"对话"

#### DroneState.msg——状态广播

```
int32 drone_id          # 无人机编号

int8[] grid_ids         # 负责的网格ID列表
float64 recent_attempt_time  # 最近尝试时间
float64 stamp           # 时间戳

# 以下仅仿真用
float32[] pos           # 位置 (x, y, z)
float32[] vel           # 速度
float32 yaw             # 偏航角
```

#### PairOpt.msg——分配请求

```
int32 from_drone_id     # 发送方ID
int32 to_drone_id       # 接收方ID

float64 stamp           # 时间戳（防重复、防乱序）
int8[] ego_ids          # 建议给发送方的网格
int8[] other_ids        # 建议给接收方的网格
```

---

## 第四阶段：代价函数（1-2天）⭐毕设重点

> **目标**：理解系统怎么给"去哪个前沿"打分，这是你毕设另一个可能改造的部分。

### 4.1 什么是代价函数？——用"打分"来理解

**打个比方**：

你面前有 3 家餐厅，你要选一家去吃。你会考虑：
1. **距离**：哪一家最近？（走路要多久）
2. **口味**：哪一家最好吃？
3. **排队**：哪一家不用等？

你会给每家餐厅打一个"综合分"，然后选分数最高的那家。

**代价函数就是这个"打分公式"**。只不过在这里，"餐厅"换成了"前沿"，"打分维度"换成了：飞过去要多久、要不要转弯、要不要掉头。

**代价越低 = 越好**（和"分数越高越好"是反过来的）。

### 4.2 RACER 的代价函数——逐行讲解

> 路径：`RACER/swarm_exploration/active_perception/src/graph_node.cpp`

这是**整个系统最核心的打分函数**，你的毕设很可能要替换它：

```cpp
double ViewNode::computeCost(
    const Vector3d& p1,         // 起点位置（当前位置）
    const Vector3d& p2,         // 终点位置（目标前沿的视点）
    const double& y1,           // 起点偏航角（当前朝向）
    const double& y2,           // 终点偏航角（目标朝向）
    const Vector3d& v1,         // 当前速度
    const double& yd1,          // 当前偏航速率
    vector<Vector3d>& path) {   // 输出：路径点

  // ===== 第1项：位置代价 =====
  // 从p1到p2需要多长时间？
  // searchPath() 会先尝试直线飞行，如果被障碍物挡住就用A*搜索
  double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_;
  //                           路径长度(米)          / 最大速度(米/秒) = 时间(秒)

  // ===== 第2项：速度方向代价 =====
  // 如果当前有速度，考虑"速度方向"和"目标方向"是否一致
  if (v1.norm() > 1e-3) {
    Vector3d dir = (p2 - p1).normalized();   // 从起点到终点的方向
    Vector3d vdir = v1.normalized();          // 当前速度的方向
    double diff = acos(vdir.dot(dir));        // 两个方向的夹角（弧度）
    // 如果当前往东飞，但目标在北边，夹角90°，就有罚分
    pos_cost += w_dir_ * diff;  // w_dir_ 默认1.5，方向不一致就加罚分
  }

  // ===== 第3项：偏航代价 =====
  // 当前朝向和目标朝向差多少？
  double diff = fabs(y2 - y1);           // 偏航角差值
  diff = min(diff, 2 * M_PI - diff);     // 取较短的旋转方向
  double yaw_cost = diff / yd_;          // 差值 / 最大偏航速率 = 需要的时间

  // ===== 第4项：取最大值 =====
  // 总代价 = max(飞行时间, 转向时间)
  // 因为无人机可以同时飞行和转向，所以总时间取决于较慢的那个
  return max(pos_cost, yaw_cost);
}
```

#### searchPath——先尝试直线，不行就A*

```cpp
double ViewNode::searchPath(const Vector3d& p1, const Vector3d& p2,
    vector<Vector3d>& path) {
  // 先尝试直线连接（最快的方式）
  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    if (map_->getInflateOccupancy(idx) == 1 || !map_->isInBox(idx)) {
      safe = false;  // 直线上有障碍物
      break;
    }
  }
  if (safe) {
    path = { p1, p2 };            // 直线可达
    return (p1 - p2).norm();     // 返回直线距离
  }

  // 直线被挡住，用A*搜索绕路
  vector<double> res = { 0.4 };  // 搜索分辨率
  for (int k = 0; k < res.size(); ++k) {
    astar_->reset();
    astar_->setResolution(res[k]);
    if (astar_->search(p1, p2) == Astar::REACH_END) {
      path = astar_->getPath();
      return astar_->pathLength(path);  // 返回A*路径长度
    }
  }

  // A*也找不到路，返回一个很大的代价
  path = { p1, p2 };
  return 100;
}
```

**用人话说**：
- 先看两点之间能不能直线飞过去（没有障碍物就行）
- 如果不行，就用 A* 算法找一条绕过障碍物的路
- 如果连路都找不到，就给一个很大的代价（表示"这条路走不通"）

### 4.3 代价函数在哪里被调用

在 `fast_exploration_manager.cpp` 中，`computeCost` 被多次调用来做决策：

```cpp
// 在 planExploreMotion() 中：
// 为每个前沿计算代价，选最小的
for (int i = 0; i < ed_->averages_.size(); ++i) {
  auto tmp_cost = ViewNode::computeCost(
      pos, ed_->points_[i],     // 当前位置 → 前沿视点
      yaw[0], ed_->yaws_[i],    // 当前偏航 → 目标偏航
      vel, yaw[1],              // 当前速度
      tmp_path);
  if (tmp_cost < min_cost) {
    min_cost = tmp_cost;
    min_cost_id = i;            // 记录代价最小的前沿
  }
}
next_pos = ed_->points_[min_cost_id];   // 下一个目标位置
next_yaw = ed_->yaws_[min_cost_id];     // 下一个目标朝向
```

### 4.4 你的毕设可能要替换什么

如果你要做的是"用 DQN 替换人工代价函数"，你需要：

1. **理解现有的 `computeCost()`**：输入是（当前位置、目标位置、当前偏航、目标偏航、当前速度），输出是一个代价值
2. **设计 DQN 的状态空间**：DQN 需要知道什么信息来做决策
   - 当前无人机的位置、速度、偏航角
   - 候选前沿的位置、大小
   - 其他无人机的位置
3. **设计 DQN 的动作空间**：从候选前沿中选一个
4. **设计奖励函数**：怎么告诉 DQN "选得好"还是"选得差"
5. **在 `planExploreMotion()` 中替换排序逻辑**：把 `computeCost()` 换成 DQN 的输出

---

## 第五阶段：轨迹规划（2-3天）

> **目标**：理解系统怎么把"去某个前沿"的决策变成一条可执行的飞行轨迹。

### 5.1 轨迹规划的整体流程

在 `fast_exploration_manager.cpp` 的 `planTrajToView()` 中：

```cpp
int FastExplorationManager::planTrajToView(
    const Vector3d& pos, const Vector3d& vel,
    const Vector3d& acc, const Vector3d& yaw,
    const Vector3d& next_pos, const double& next_yaw) {

  // 1. 计算偏航角的最小时间
  double diff0 = next_yaw - yaw[0];
  double diff1 = fabs(diff0);
  double time_lb = min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;

  // 2. 用A*搜索几何路径
  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(pos, next_pos, optimistic)
      != Astar::REACH_END) {
    ROS_ERROR("No path to next viewpoint");
    return FAIL;
  }
  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  shortenPath(ed_->path_next_goal_);  // 缩短路径（去掉冗余点）

  // 3. 根据距离选择不同的规划策略
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len < radius_close || optimistic) {
    // 很近：直接用路径点优化
    planner_manager_->planExploreTraj(
        ed_->path_next_goal_, vel, acc, time_lb);
  } else if (len > radius_far) {
    // 很远：先飞到中间点
    planner_manager_->planExploreTraj(
        truncated_path, vel, acc, time_lb);
  } else {
    // 中等距离：用运动学A*搜索
    planner_manager_->kinodynamicReplan(
        pos, vel, acc, next_pos, ...);
  }

  // 4. 规划偏航轨迹
  planner_manager_->planYawExplore(
      yaw, next_yaw, true, ep_->relax_time_);

  return SUCCEED;
}
```

### 5.2 A* 搜索——找最短路径的基础算法

> 路径：`RACER/swarm_exploration/path_searching/src/astar2.cpp`

**什么是 A* 算法？**

你在迷宫里找出口，A* 就是一种非常聪明的找路方法：
- 它会同时考虑"已经走了多远"（g值）和"离出口大概还有多远"（h值）
- 优先探索"看起来最有希望"的方向
- 保证找到最短路径

**在 RACER 里**：A* 用来找"从当前位置到目标前沿"的无碰撞折线路径。

### 5.3 B-Spline 轨迹优化——让飞行更平滑

> 路径：`RACER/swarm_exploration/bspline_opt/src/bspline_optimizer.cpp`

**为什么要优化？**

A* 找到的路径是一条"折线"——直角转弯、生硬的路径点。但无人机不能急转弯（物理上做不到），所以我们需要把折线变成平滑的曲线。

**B-Spline 优化的目标**：

| 优化目标 | 含义 |
|----------|------|
| 平滑性 | 轨迹不能有急转弯（jerk 最小化） |
| 可行性 | 速度和加速度不能超过无人机的极限 |
| 障碍物距离 | 不能离障碍物太近 |
| 集群避碰 | 不能和其他无人机撞上 |

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
| Odometry | 里程计 | 无人机对自己位置的估计 |
| Ray Casting | 射线投射 | 沿一条线检测碰撞 |
| BFS | 广度优先搜索 | 一种图搜索算法 |

---

## 附录B：你的毕设改造路线图

假设你的毕设方向是"用深度强化学习优化无人机集群探索"，以下是建议的改造路线：

### 第一步：跑通现有系统（1周）

1. 按照项目 README 编译和运行
2. 在 Rviz 里触发探索，观察 5 架无人机如何协同工作
3. 用 `rostopic echo /swarm_expl/drone_state` 查看无人机状态广播
4. 用 `rqt_graph` 查看节点关系图

### 第二步：深入理解核心代码（2周）

按照本文档的阅读顺序，重点精读：

| 文件 | 关注点 | 文件行数 |
|------|--------|----------|
| `expl_data.h` | DroneState, ExplorationData 结构体 | ~120行 |
| `fast_exploration_fsm.cpp` | FSMCallback, optTimerCallback | ~900行 |
| `frontier_finder.cpp` | searchFrontiers, expandFrontier | ~500行 |
| `graph_node.cpp` | computeCost 代价函数 | ~100行 |
| `hgrid.cpp` | updateGridData 网格更新 | ~400行 |
| `fast_exploration_manager.cpp` | allocateGrids, findGridAndFrontierPath | ~800行 |

### 第三步：设计你的改进方案（1周）

1. 确定你要替换哪个模块（分配 or 代价函数 or 两者都换）
2. 画出修改方案的流程图
3. 设计 DQN 的状态、动作、奖励

### 第四步：实现和测试（3-4周）

1. 在现有代码框架里插入你的算法
2. 先用少量无人机（2-3架）测试
3. 逐步增加到 5 架
4. 对比你改进前后的探索效率（覆盖时间、路径长度等）

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
6. **代码行号是你的导航**——本文档标注了每个关键函数的文件位置，直接跳过去看

祝你的毕设顺利！🎓
