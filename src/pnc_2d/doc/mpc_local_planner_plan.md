# MPC 局部规划实施方案（pnc_2d）

> 状态：**P5.0~P5.3 已完成并验证**（2026-09-22）；剩 P5.4（仿真 E2E + 调参）。
> 相关：`pnc2d_restructure_plan.md`（整改总计划、P3/P4 上下文）、`pnc2d_dev_plan.md`（全局侧细节）。
> 范围：**只动 `src/pnc_2d`**；感知只通过话题消费，不改 `perception`。
>
> **参考实现（2026-09-22 修订）**：以 **`/home/gmd/SLAM-PNC/PNC` 的 `MpcController`**
> （`src/controller/src/controller/mpc.cpp`）为准 —— 阿克曼模型 + `(a, δ)` + OSQP。
> 本仓库 `scripts/mpc.py` 是**早期原型，不作为基准**（仅保留供离线玩耍）。
> 两者的差异与必要的映射见 §2.1。
> 推荐算法推导/调参见 `mpc_nmpc_guide.md`（但其中的 condensing 写法与本次实现不同，见 §2.2）。

---

## 0. 决策定案（2026-09-22）

| # | 问题 | 定案 | 理由/备注 |
|---|---|---|---|
| D1 | 局部规划算法 | **MPC（单套）**，不做"路网一套、非路网一套" | 两模式的差别是**同一优化问题的不同约束集**，不是两种算法 |
| D2 | 模式切换方式 | **两套 profile：`route` / `free`**，由**数据驱动**判定：`setCorridor()` 非空 ⇒ `route`，否则 `free` | 不做多余的 `local.profile` 参数；走廊本身就是"我在路网模式"的充分标志 |
| D3 | 距离场来源 | **`grid_map/esdf_2d`（点云）→ 栅格化为本地距离图（双线性插值）**；自算 EDT **作兜底/一致性校验** | 它的 `esdf_unknown_as_occupied=false`（乐观）正是我们要的语义；不重复实现 ESDF |
| D4 | 硬碰撞判定 | **`grid_map/occupancy_inflate_2d`（0.10 m 全分辨）** | `esdf_2d` 抽点（`esdf_pub_step=2` → 0.20 m）且**不发障碍本体**（`d≤0` 不发），硬判定不能靠它 |
| D5 | 走廊严格度 | **硬约束，不可违反**（方案 A）：路网模式遇障 ⇒ `BLOCKED` 停车 | 用户确认；`corridor_width` 是静态通行性承诺 |
| D6 | 避障能力 | **v1 反应式**（只看当前帧距离场） | 动态障碍速度/轨迹估计列 §7 未来优化点 |
| D7 | 求解器 | **OSQP（0.6.2，C API）+ **结构固定 + 热启动****；已装 `ros-humble-osqp-vendor` | 比参考实现更进一步：它每周期 new 一个 Solver（等于冷启动），我们复用工作区 |
| D8 | 降级链 | `MPC 成功 → 用` / `求解失败或超时 → 停车 + 报错`；**不做 pure_pursuit 兜底** | 用户确认；停车比"凑合走"可预测 |
| D9 | 局部状态枚举 | **独立枚举 `LocalStatus`**（不复用 `PlannerStatus`） | 局部状态语义不同（跟踪/到达/阻塞） |
| D10 | `producesCmdVel()` | **保留** | Null 声明"我不产生速度"，manager 据此决定是否转发 |
| D17 | **参考实现**（2026-09-22） | **SLAM-PNC 的 `MpcController`**；`scripts/mpc.py` 仅作历史原型 | 用户指定；其稀疏 QP 写法避开了 condensing 类 bug（见 §2.2） |

---

## 1. 为什么是 MPC

| 算法 | 路网贴线 | 自由避障 | CPU | 依赖 | 结论 |
|---|---|---|---|---|---|
| pure_pursuit / Stanley | 好（但会切角） | ❌ 本质不具备 | 极低 | 无 | **不做**（D8） |
| **MPC（运动学 + 距离场）** | ✅ 横向偏差可写成硬约束 | ✅ 时域内主动避让 | 中 | OSQP | **采用** |
| DWA | 一般（采样抖） | ✅ | 低 | — | 与"严格贴线"冲突 |
| TEB | ✅ | ✅ | 中高 | g2o | 调参成本高、依赖重 |
| MINCO / SFC | 要把路网表达成约束面 | ✅✅ | 高 | 重 | 2D 地跑 + 人工路网属过度设计 |

**ESDF vs costmap（结论）**：MPC 需要的不是"某个地图话题"，而是**到障碍的距离作为决策变量的函数**（QP 里要把 `d(p_k) ≥ r_safe` 一阶线性化）。纯 costmap 的梯度没有意义，只适合采样类方法；而 **costmap 做一次欧氏距离变换就是 ESDF**——同一份数据多算一步。因此问题只是"距离从哪来"，见 D3/D4。

---

## 2. 问题形式化

**模型**（差分驱动，Go2 用 (v, ω) 接口）：

```
x_{k+1} = x_k + v_k·cos(θ_k)·dt
y_{k+1} = y_k + v_k·sin(θ_k)·dt
θ_{k+1} = θ_k + ω_k·dt
```

**时域**：`N = 15`，`dt = 0.1 s`（1.5 s 前瞻）；控制量 `u = [v, ω]`（v ≥ 0，不倒车）。

**代价**（两 profile 共用骨架，权重不同）：

```
J = Σ_k [ w_ey·e_y(k)² + w_eth·e_θ(k)² + w_v·(v_k − v_ref(k))² + w_du·(Δu_k)² ]
  + w_term·(终点进度/距离项)
```

**约束**：

| 约束 | `route` 模式 | `free` 模式 |
|---|---|---|
| 横向偏差 | **硬**：`|e_y(k)| ≤ corridor_width`（0 = 严格贴线） | 软：跟踪全局路径偏差 |
| 障碍距离 | 硬（兜底）：不侵入 `occupancy_inflate_2d` | **软**：`d_esdf(p_k)` 惩罚（越近越大）+ 紧急硬阈值 |
| 速度 | `v ≤ min(edge.speed_limit, 限速区, v_max)` | `v ≤ v_max` |
| 控制 | `v ∈ [0, v_max]`，`ω ∈ [−ω_max, ω_max]`，`|Δv| ≤ a_max·dt`，`|Δω| ≤ α_max·dt` | 同左 |

**未知/视野外**：走**软代价**（用 `esdf_unknown_as_occupied=false` 的场），否则 4 m 视野边界会被当成墙，机器人不敢走。结构化占据格才是硬约束。

**`route` 模式的 `BLOCKED` 判定**：若在时域内不存在满足走廊硬约束与障碍硬约束的解 ⇒ 返回 `kBlocked`，`cmd = 0`，交状态机（D5）。

### 2.1 从 SLAM-PNC（阿克曼）到本仓库（差速）的映射 —— **实际实现采用的形式**

参考实现是阿克曼：`s = [x, y, θ, v]`，`u = [a, δ]`（前轮转角），且 `ω = v/L·tan δ`。
Go2 是差速 `(v, ω)` 接口、**没有前轮转角**，所以取：

```
状态 s = [x, y, θ, v]        控制 u = [a, ω]
ẋ = v·cosθ    ẏ = v·sinθ    θ̇ = ω    v̇ = a        （ω 在运动学里扮演 δ 的角色）
A = I + dt·∂f/∂s（绕参考点线性化）    B = dt·∂f/∂u
  A：位置对 (yaw, v) 有偏导；θ 不依赖 v（bicycle 里 θ̇ 依赖 v，差速里不依赖）
  B：B(2,1)=dt（ω 直接进 θ̇）  B(3,0)=dt（a 直接进 v̇）
```

两处**新增**（参考实现的 MPC 只做轨迹跟踪，它假设上游 trajopt 给的轨迹已避障）：

| 新增 | 形式 | 为什么这么写 |
|---|---|---|
| **走廊硬约束** | `|n_k·e_k[0:2]| ≤ half_width`，`n=(-sinθ_ref, cosθ_ref)` | 横向偏差对决策变量是**线性**的，所以能直接当硬约束；不必"软化" |
| **障碍软代价** | `d ≈ d_ref + ∇d·e` 代入 `w·(r_safe−d)²`，展开成 **rank-1 二次项 + 线性项** | 保持**凸性**（`∇d∇d'` 半正定）；非凸障碍约束会把 QP 搞成不可解 |
| **障碍硬下界** | `d_ref + ∇d·e ≥ d_hard`（线性化）+ **解后用真实距离复核** | 前者让优化器主动避让，后者是"绝不输出会撞的轨迹"的保证 |

### 2.2 为什么用"误差 + 稀疏 QP"而不是 condensing

决策变量取 **z = [e_0..e_N, Δu_0..Δu_{N-1}]**（`e = s − s_ref` 是**世界系绝对误差**，
`Δu = u − u_ref`），动力学作为**等式约束**：

```
e_0 = e_init;   e_{k+1} = A_k·e_k + B_k·Δu_k
```

于是代价 `Σ e'Qe + Σ Δu'RΔu + Σ (Δu 变化率)` 全是二次型，**gradient = 0**。

这一点是刻意选的：参考轨迹的前馈作用**隐含在"e 是相对参考的绝对误差"里**，不需要任何
显式前馈项 —— 也就不会出现 `scripts/mpc.py` 文档里列的那类经典 bug（"把绝对 u 当偏差
代入、丢了 `[v_ref, 0, w_ref]` 前馈 → e=0 时解出 u=0 → 机器人原地不动"）。

代价是状态维度进 QP（变量数 `4(N+1) + 2N`），但矩阵**稀疏**且**结构每周期完全不变**，
所以能真正复用 OSQP 工作区做热启动。

---

## 3. 输入与距离场

| 输入 | 话题 | 用途 | 说明 |
|---|---|---|---|
| 局部 ESDF | `grid_map/esdf_2d`（`PointCloud2`, `intensity` = 距离 m） | 软代价/引导 | `esdf_max_dist=3.0 m` 截断；`esdf_pub_step=2` → 0.20 m 抽点；**不含障碍本体**（`d≤0` 不发） |
| 局部膨胀图 | `grid_map/occupancy_inflate_2d`（`OccupancyGrid`, 0.10 m） | **硬碰撞判定**、`BLOCKED` 判定 | 全分辨、含障碍本体 |
| 全局图 | `global_map/occupancy`（latched） | 静态兜底 | 已有 |
| 位姿 | `lightning/perception/pose`（best_effort） | 状态 | 已有 |
| 全局路径/走廊 | `PlanPath` service 返回（P4） | 参考轨迹 + 走廊 | `route` 模式带走廊折线与 `corridor_width` |

**栅格化**：把 `esdf_2d` 的点按索引塞进"本地图范围"的网格（`O(点数)`），空格视为 `≥ max_dist`（够远），查询用双线性插值 → **既用了感知的结果、又得到网格的光滑性**（点云最近邻查询得到的距离函数是分段不光滑的，线性化会抖）。

**时间对齐与降级**（参数化）：
- 缓存最近一帧 ESDF；`local_mpc.esdf_timeout`（默认 0.3 s）内视为有效；
- 超时 ⇒ 只用 `occupancy_inflate_2d` 硬判定 + 保守减速（并把状态标 `kDegraded`，日志 WARN）；
- 若 `occupancy_inflate_2d` 也缺失 ⇒ 停车报错。

**一致性校验**（单测里）：自算 EDT（复用 `core/clearance_field.hpp`）与栅格化 esdf 对比，偏差应在半格量级内 —— 顺带保证我们对它的语义理解正确。

### 3.1 ESDF 原理与开销（"到最近障碍的距离"为什么一点也不贵）

**它是什么**：每个格子的值 = **到最近障碍（占据格）的距离**（完整定义是带符号：障碍外为正、内部为负；2D 占据图里通常只用非负"到最近致命格的距离"）。三种用途：避障代价/约束（`d ≥ r_safe`）、**梯度引导**（$\nabla d$ 指向远离障碍，可直接线性化）、连续碰撞检查（查距离，不必逐格扫）。

本仓库有两份、**同一个算法**：perception 的 `md_.esdf2d_`（发布为 `grid_map/esdf_2d`）与 pnc_2d 的 `ClearanceField`。

**朴素做法为什么不可行**：每格遍历所有障碍是 $O(N \times M)$。真实站点图 186k 格 × 12k 障碍格 ≈ **22 亿次**。

**实际算法：可分离 + 抛物线下包络**（Felzenszwalb & Huttenlocher, 2012）。关键洞察是欧氏距离的平方在 x/y 上可分离：

$$
D^2(x,y)=\min_{(p_x,p_y)\in\text{障碍}}\big[(x-p_x)^2+(y-p_y)^2\big]
=\min_{p_y}\Big[\underbrace{\min_{p_x}(x-p_x)^2}_{f_0(p_y)\ \text{——行变换}}+(y-p_y)^2\Big]
$$

于是变成两趟 1D 距离变换（见 `src/perception/src/grid_map.cpp::build2DESDF()`）：

```cpp
for (iy…) edt1D(edt_f_ 的第 iy 行) → edt_sq_;   // 行方向：水平最近障碍
for (ix…) edt1D(edt_sq_ 的第 ix 列) → esdf2d_[.] = sqrt(min(·, max_sq)) / inv_res;
```

单趟 `edt1D` 是**抛物线求下包络**：把每个候选 $f(p)+(q-p)^2$ 看成抛物线，维护下包络（代码里的 `edt_v_` = 顶点、`edt_z_` = 交点）。任意两条抛物线最多相交一次 ⇒ 用栈弹出即可，**均摊 $O(n)$**：

```cpp
int k = 0; v[0] = 0; z[0] = -INF; z[1] = +INF;
for (int q = 1; q < n; ++q) {
  float s;
  while (true) {                                   // 弹出被支配的抛物线
    s = ((f[q] + q*q) - (f[v[k]] + v[k]*v[k])) / (2.f * (q - v[k]));
    if (s > z[k]) break;
    --k;
  }
  ++k; v[k] = q; z[k] = s; z[k+1] = +INF;
}
k = 0;
for (int q = 0; q < n; ++q) {                      // 取值
  while (z[k+1] < q) ++k;
  d[q] = (q - v[k])*(q - v[k]) + f[v[k]];
}
```

⇒ 总体 **$O(N)$、精确欧氏距离**（不是 chamfer/倒角近似），与障碍数量无关。

**三个进一步降本的手段（本实现都用了）**

| 手段 | 代码 | 效果 |
|---|---|---|
| **截断** `esdf_max_dist = 3.0 m` | `sqrt(std::min(v, max_sq))` | 抛物线影响范围有界、远处饱和；发布带宽也小 |
| **惰性 + 节流** | `esdf2d_stale_` 脏标记 + `pub_2d_interval`；且**只在有人订阅时才构建**（`get_subscription_count() > 0`） | 没变化不算；不订阅不花感知的 CPU |
| **局部小图** | 局部图只有 4 m 范围 | 1.6k 格 → 十几 µs |

**实测（2026-09-22，本机用 pnc_2d 的 `ClearanceField`，与感知同算法）**

| 规模 | 耗时 |
|---|---|
| 全局 607×307 @0.05 m（186k 格，6.5% 障碍） | **4.76 ms** |
| 同上，30% 障碍（更碎） | 5.25 ms |
| 局部 4 m×4 m @0.10 m（1.6k 格） | **0.018 ms** |
| 局部 4 m×4 m @0.05 m（6.4k 格） | 0.120 ms |

复现方式（30 行，不依赖 ROS）：编译 `clearance_field.cpp + cost_map_2d.cpp` 后对随机障碍图调 `ClearanceField::build()` 计时即可。
结论：整张站点图 ~5 ms、局部图 ~20 µs，相对 20 Hz（50 ms）预算完全不是负担 ⇒ **"自算 EDT 作兜底"没有性能顾虑**；而"直接用感知发布的 esdf" 省掉的是这份**重复计算**与语义分歧。

**四个容易踩的细节**

1. **用 d 还是 d²**：EDT 内部算的是 $d^2$，最后 `sqrt` 成米。代价函数用 $d^2$ 更省更平滑；MPC 做线性化约束时用 $d$ 更直观 —— P5.1 里定。
2. **半格系统偏差**：`ClearanceField::distanceToLethal` 是"到最近致命格**中心**"的距离，天然有半格偏差；它给出的 `lowerBoundM()/upperBoundM()` 就是为这个偏差包的上下界（footprint 快路径用它做保守判定，不会误判"安全")。
3. **未知格算不算障碍**：`esdf_unknown_as_occupied`（默认 **false**）。这一条直接决定机器人会不会被局部视野边界"困住"——MPC 必须用乐观语义（§2）。
4. **增量更新**：ego-planner 系通常只重算脏区/滑动窗口，我们是整图重算（5 ms 可接受）。要再省可加脏区，列入 §7 优化点。

---

## 4. 接口与话题

```cpp
// core/local_planner.hpp（P3 定义，P5 实现）
enum class LocalStatus { kIdle, kFollowing, kGoalReached, kBlocked, kDegraded, kFailed };

struct Twist2D { double v{0.0}, w{0.0}; };

struct LocalPlanResult {
  LocalStatus status{LocalStatus::kIdle};
  Twist2D cmd;
  double progress{0.0};      // 0~1 路径进度
  double cross_track{0.0};   // 当前横向偏差 [m]
  double time_to_goal{0.0};  // 预估剩余时间 [s]
  double solve_ms{0.0};      // 求解耗时（诊断）
  int    solver_iter{0};
};

class LocalPlanner {
public:
  virtual std::string type() const = 0;
  virtual bool configure(const ParamReader &) = 0;
  virtual void setGlobalPlan(const std::vector<Pose2D> &path, bool has_yaw) = 0;
  virtual void setCorridor(const RouteCorridor *corridor) = 0;   // 非空 ⇒ route profile
  virtual void setSpeedLimit(double v_limit) = 0;                // 限速区
  virtual void setCostMap(std::shared_ptr<const CostMap2D> local_inflated) = 0;
  virtual void setDistanceField(const LocalDistanceField *field) = 0;  // ESDF 栅格化结果
  virtual void setDynamicObstacles(const std::vector<DynamicObstacle> &) = 0;  // v1 收而不用
  virtual LocalPlanResult computeCommand(const Pose2D &pose, double dt) = 0;
  virtual void reset() = 0;
  virtual bool producesCmdVel() const = 0;
};
```

| 话题/服务 | 类型 | 方向 |
|---|---|---|
| `/pnc_2d/cmd_vel` | `geometry_msgs/Twist` | local → `quadropted_controller`（经 manager 转发/仲裁） |
| `/pnc_2d/local_status` | `pnc_2d/LocalStatus`（新 msg，latched） | local → 监控/状态机 |
| `~/switch_planner` | `pnc_2d/SwitchPlanner` | 复用现有：`local.type` 热切换 |
| `~/reload_params` | `std_srvs/Trigger` | 复用现有：改权重/时域不用重启 |

---

## 5. 实施步骤与验收

### P5.0 工具链（✅ 已完成 2026-09-22）
`ros-humble-osqp-vendor` 已装。

⚠ **CMake 与 API 的两个真坑（已踩）**：
1. 这套 vendor **没有** `osqp` imported target，`${osqp_vendor_INCLUDE_DIRS}` 也可能是空 →
   只靠“target 存在就 link”会漏 include 目录（`osqp.h` 找不到）。
   可靠做法：`find_path(OSQP_INCLUDE_DIR osqp.h …)` + `find_library(OSQP_LIBRARY osqp …)`，
   拿不到就 `FATAL_ERROR` 提示装包。
2. 头文件是 **0.5 风格接口**（`osqp_setup(&work, const OSQPData*, &settings)`），
   **不是** 0.6 的 `(P, q, A, l, u, m, n)` —— 尽管 `OSQP_VERSION` 宏写着 0.6.2。
   以头文件为准；`osqp_update_P_A` 还多两个 `*_idx` 参数（传 `OSQP_NULL` = 按原顺序全量更新）。

### P5.1 库层 MPC（✅ 已完成 2026-09-22，ROS-free）
交付：`core/local_distance_field.{hpp,cpp}`、`local/mpc_local_planner.{hpp,cpp}`、
`test/test_distance_field.cpp`、`test/test_mpc_local_planner.cpp`、`config/local_mpc.yaml`。

**验收与实测（`./build/pnc_2d/test_mpc_local_planner`）**

| 验收项 | 要求 | 实测 |
|---|---|---|
| 直线跟踪 | RMSE < 0.05 m | **全局 RMSE 0.0484 m**（含 0.30 m 初始偏差的收敛）；**稳态 RMSE 0.0002 m**；末态 0.0000 m |
| 圆弧跟踪 | 横向误差 < 0.10 m | **RMSE 0.0099 m**，最大 0.0636 m（R=3 m，400 步闭环） |
| **走廊硬约束** | 1000 组扰动越界 = 0 | **1000/1000 出解、越界 0**；预测横向最大 0.3366 m（限 0.50） |
| 求解耗时 | P99 < 20 ms | **P50 0.132 ms / P99 2.088 ms / max 2.427 ms**（25 次迭代；QP 94 变量 / 182 约束） |
| 降级 | 无距离场 → 限速 + 报 kDegraded | ✅ `v ≤ 0.30 m/s`，状态 `DEGRADED` |
| 无解处理 | `kBlocked` + `cmd = 0` | ✅ 贴边且航向朝外 → `primal infeasible` → `BLOCKED`，cmd 严格为 0 |
| 制动/到位 | 不冲过终点 | 从 1.0 m/s 接近：末速度 0.260 m/s，最远 x=9.959（不超调） |
| 避障 | 硬下界不被侵入 | 最小预测距离 0.2490 m（下界 0.25）；软代价方向正确（离开障碍） |
| 距离场插值 | 误差 ≤ 半格量级 | 0.2 m 采样子 0.1 m 栅格：511 点最大误差 **0.0805 m**（比“最近邻填充”的 0.15 m 好一倍） |

**单测规模**：`test_distance_field` 7 用例 + `test_mpc_local_planner` 15 用例；
全包 **8 个测试程序 / 95 用例 / 0 失败**（`colcon test-result` 汇总 103 项）。

**开发中抓到的 3 个真 bug（都不是“调参问题”）**
1. **`P` 里的变化率项写成了下三角** → OSQP 直接拒绝（`P is not upper triangular`）。
   现已加建表自检（下三角 → `abort` 并打印位置）。
2. **动力学等式漏了 `A` 的单位阵项**（只写了 `-A(i,2/3)·e`，漏了 `-e_k(i)`）→
   约束被放开，求解器还报成功，但**预测轨迹瞬移**（第 1 步就跳到参考线上）。
   这是最危险的一类（拿着假轨迹去发指令），现加了两道防线：
   ① `validateSolution` 算动力学残差（正常 ~1e-14，> 1e-3 直接拒发指令）；
   ② `dumpQp()` 可把 H/q/A/l/u 导成文本。
3. **距离场填充用“复制最近邻”** → 阶梯场，插值误差 0.15 m（安全距离才 0.45 m）。
   改成“可用邻居取平均 + 逐轮外扩”（调和填充，在采样缝里给出线性过渡）→ 误差降到 0.08 m。

### P5.2 参考一致性校验（范围已调整）
**不再**跟 `scripts/mpc.py` 对比（用户 2026-09-22：那个原型不作为参考）。改为：
- 与**解析真值**对比：直线/圆弧的闭环误差（已在 P5.1 覆盖）；
- 与 **SLAM-PNC 参考实现**对比：同参数下把 `MpcController` 的 QP 矩阵（H/q/A/l/u）导出，
  与我们的矩阵做**逐项对比**（同样是无障碍场景），验证模型/离散化/约束组装一致。
  这一项比“误差差 5%”更有价值：它能在“两条轨迹碰巧都能跑”的情况下抓出矩阵写错。
- 结果记入本文档 §9。

### P5.3 节点接线（✅ 已完成 2026-09-22）

`local_planner_node` 订阅局部图 + 距离场 → 喂给算法；`local.type: mpc|none` 热切换。

**P5.3a：感知输入接线**

| 输入 | 话题（实测确认） | 处理 |
|---|---|---|
| 局部膨胀图 | `grid_map/occupancy_inflate_2d`：`OccupancyGrid`，80×80 @ 0.1 m **滑窗**，frame `map`，reliable+depth1+volatile | → `CostMap2D` → `setCostMap()` |
| 距离场 | `grid_map/esdf_2d`：`PointCloud2`，`point_step=**32**`、`intensity` 偏移 **16**、`width=1598/height=1` | 按**字段名**迭代解析 → `LocalDistanceField::buildFromSamples()` → `setDistanceField()` |
| 底盘速度 | `topics.odom`.`twist` | → `setCurrentVelocity(v, w)` |

★ **空点云 ≠ 拿不到距离场**。感知只发 `0 < d ≤ esdf_max_dist` 的格，周围没有障碍时
点云就是空的。含混处理的代价是车在开阔大路上以 0.3 m/s 爬行，而日志看起来像"感知挂
了"。所以新增 `LocalDistanceField::buildFree()`：**场有效，但每个格子都是
`max_distance`**。

★ **必须用字段名解析，不能硬编码偏移**。PCL 的 `PointXYZI` 经 `PCL_ADD_POINT4D` 把前
4 个 float 对齐，实测 `intensity` 在 **16** 而不是 12、`point_step` 是 **32** 而不是
16。硬编码偏移会读到 padding（全 0）→ 距离场全 0 → 车"处处是障碍"，而且**不报错**。

距离场超时 `local.esdf_timeout`（默认 0.3 s，感知 10 Hz 发）→ `setDistanceField(nullptr)`
→ 降级限速；恢复后挂回去。**不允许**用过期场做硬约束（人走了、车动了，障碍位置早就
变了）。同时订阅本身就是"让感知开始算距离场"的开关（感知只在
`get_subscription_count() > 0` 时计算，实测订阅前 `Subscription count: 0`）。

**P5.3b：走廊端到端接通**

四跳：`RouteNetworkPlanner` → `PlanPath.srv` → `pnc_manager` → `FollowPath.action` → 局部。
语义：**path 即走廊中心线**，允许横向偏离 ±`corridor_half_width`（接收端
`local_planner_node.cpp` 本来就这么写，缺的只是发送端）。跨多条通道取最严的（半宽
min、限速 min）。

⚠ 这一跳没接通时**完全看不出来**：路径仍然沿通道算，只是局部拿不到宽度 → 当自由空间
跟 → "贴线走"静默失效。所以 E2E 用两处独立证据（manager 日志的走廊摘要 + 局部状态
`route_mode`）。

**执行中发现并修掉的 4 个问题**

| # | 问题 | 现象 | 修法 |
|---|---|---|---|
| 1 | `local.esdf_fill_radius: 2` 用 `paramDouble` 读 | **节点启动即崩**：`ParameterTypeException: expected [double] got [integer]`。ROS 2 参数带类型，yaml 写 `2` 就是 integer | 新增 `paramInt()` |
| 2 | 没声明 `sensor_msgs` 依赖 | 编译期 `point_cloud2.hpp: 没有那个文件` | `package.xml` + `find_package` + `ament_target_dependencies` |
| 3 | 滑窗原点每帧都变 → "几何变化"告警 | 真实驾驶时日志被永久刷屏 | 只在**尺寸/分辨率**变化（真异常）时告警 |
| 4 | 测试里"车"不动 | 管理器**卡住判据**（10 s 位移 < 0.2 m）取消任务，后续阶段全跑不到 | 测试按 `cmd_vel` 做差速运动学积分（顺带把 `setCurrentVelocity` 也走到） |

**验收（E2E-6 `test/e2e/test_mpc_wiring.py`，22 项全通过）**

| 阶段 | 断言 | 实测 |
|---|---|---|
| A 无数据 | 局部 `DEGRADED`（不是 FOLLOWING）+ 限速 | max\|v\| = 0.100 ≤ 0.30 |
| B 只给图不给场 | 仍 `DEGRADED`（图不能替代场） | max\|v\| = 0.100 |
| C 空点云 | **`FOLLOWING`**、提速、真发 `cmd_vel`、闭环前进 | max\|v\| = 1.000，58 条，前进 3.05 m |
| D 墙横在路上 | 行为必须变化 | `BLOCKED`，消息 `预测轨迹会侵入障碍硬距离（最小 0.000000 m < 0.250000 m）` |
| E 日志 | 解析器认得出真实布局 | 采样数序列 `[0, 0, 200]` |
| F 反例 `none` | 无发布者、无消息 | 0 / 0 |
| G route 模式 | 走廊一路传到局部 | `route_mode=true`，半宽 0.60 m（= `routes.yaml`） |

求解耗时最大 0.34 ms（指标 < 20 ms，余量充足）。

### P5.4 仿真 E2E + 调参 + 文档

**自由空间验收（✅ 2026-09-22，`test/sim/test_drive_goal.py`，6/6 通过）**

| 项 | 标准 | 实测 |
|---|---|---|
| 到点停车误差 | ≤ 0.15 m | **0.017 / 0.043 m**（`local.goal_tolerance=0.08`） |
| 末速（位姿差分） | ≈ 0 | **0.012~0.013 m/s** |
| 横向误差（全程，含 90° 起转） | ≤ 0.30 m | **0.051~0.058 m** |
| 横向误差（稳态） | ≤ 0.10 m | **0.016~0.026 m**（均值偏置 ±0.001~0.006） |
| 无碰撞 | 净距 > 0 | ≥1.0 m（探针只搜 ±2 m，已饱和） |
| 指令不越界 | ≤ v_max/w_max | ✓ |
| 被控对象保真度 | — | **0.93~0.96**（稳态段路程 / 指令积分） |

14.5~14.8 s 走完 3.75 m（≈0.26 m/s，参考速度 0.35、底盘上限 0.42）。

> ⚠ **到点误差只有在 `local.goal_tolerance` 严格小于验收阈值时才有意义**。踩过：
> 容差原来也设 0.15，于是管理器在 `remaining ≤ 0.15` 就停机，量到的“到点误差”
> 恒等于它自己的触发条件（一次量到 0.141 m，看着合格，其实什么都没证明）。
> 收紧到 0.08 后残差降到 0.017~0.043 m ⇒ 这才是“控制器真能收敛到哪”。

**贴线（route）模式验收（✅ 2026-09-22，`test/sim/test_route_lane.py`，7/7 通过）**

测试自带一条 `corridor_width: 0.0` 的临时直通道（站点通道是 0.60 m 宽，验不出"严格"），
沿车头方向生成 ⇒ 贴线误差可以直接与已知折线比：

| 项 | 标准 | 实测 |
|---|---|---|
| 贴线偏差（全程） | ≤ 0.10 m（含起步对线的一次外摆） | **0.048 m** |
| 贴线偏差（稳态） | ≤ 0.05 m（= `corridor_min_tolerance` 硬界） | **0.025 m**（均值偏置 +0.004） |
| 到点停车误差 | ≤ 0.15 m | **0.017 m** |
| `route_mode`（走廊真的传到局部） | true | ✓ |
| 无碰撞 / 指令不越界 | ✓ | ≥1.0 m / ✓ |

**在仿真里定位到的 5 个真问题**（都不是调参问题，都是先量后改）

| # | 问题 | 症状与证据 | 修法 |
|---|---|---|---|
| 1 | **硬状态约束没考虑"从当前状态的可达集"** | `v_min ≤ v_k ≤ v_upper` 是硬约束，但 `v_0 = v_now` 不受约束。`v_now` 一旦跑出这条带子，QP 就被要求**一步内**吃下差值，超过 `a_max` 就是 `primal infeasible`。两个方向都踩了：① 步态瞬态冲到 0.89 m/s（限速 0.35）⇒ 上界；② 定位 twist 噪声读到 **−0.23**（`v_min=0`）⇒ 下界。日志特征：`v_now=-0.230 e_v0=-0.580 | maximum iterations reached`，而同一条日志里 `v_now=+0.090` 的周期只要 25 次迭代 | 两个界都取"可达值"：`u = max(v_upper, v_now − a_max·dt·k)`、`l = min(v_min, v_now + a_max·dt·k)`。正常情况完全不变；传感器再离谱也不会把 QP 顶死。回归：`SolverStaysFeasibleWhen{FasterThanLimit,SlowerThanMin}` |
| 2 | **速度反馈（`odom.twist`）低速不可用** | 本仓库 lightning 的 twist：低速时噪声 ±0.1、**均值都偏低**（车实际走 0.045 m/s 时读到 −0.03）。而 MPC 的命令结构上是 `v_cmd ≤ v_now + a_max·dt`（模型里速度不能瞬变）⇒ 反馈不准就**永远起不来**：cmd 卡在 0.06，60 s 只挪 3 m | 节点新增 `local.vel_from_pose`：用 **0.25 s 窗口的位姿差分**算车体速度 + 一阶低通（帧间差分会把位置噪声放大 σ/dt，必须用窗口）。位姿本身是可靠的（跟踪指标都由它算） |
| 3 | **禁行区膨胀被算了两次** | `map_server` 按**外接半径 0.472 m** 烧进全局图，规划器又做一遍 footprint 检查 ⇒ 实际禁入带 ≈0.7~1.0 m。车停在禁行区外 0.72 m（合法位置）时，全局规划直接 `PlanFail[起点处车体轮廓与障碍重叠]` ⇒ **任何目标都无法起步**，看起来像控制器坏了 | `zones.inflate: 0.0`（用户决定）：只靠 footprint 检查这一次精确判定。画线器 `--zone-inflate` 同步改。另给 map_server 补 on-set 回调（修 `param set` 改了参数服务却没改成员的老陷阱） |
| 4 | **底盘会保持最后一条指令** | 任务结束/进程退出时若不再发 cmd_vel，车会以最后那个速度**一直走**（实测我的测试被中断后车自己跑掉，只能 teleop 追回）。同时也是验收项"到点末速 0.264 m/s"的真因 | `finish()`/`endAsCanceled()`/`rclcpp::on_shutdown`/析构 里都发零速。**底盘侧仍应加 cmd_vel 超时看门狗**（更根本的兜底，属 `bringup/`） |
| 5 | **参考折线的切线在毫米级基线上算成了噪声** | `rebuildReference` 用**相邻两点**算切线：`atan2(Δy, Δx)`。贴线模式下路径前两点重合（车的位置 = 通道起点，相距 ~1e-5 m）⇒ 方向完全由数值噪声决定：实测 `ref_yaw[0]` 差了 ~90°，`e_ψ(0) = −96.7°`、`curv = 0.546 1/m`（一条直线上！）。后果：一加速就因航向误差产生横向偏移、撞严格走廊的硬界，代价上"不动"更便宜 ⇒ **MPC 趴窝**（`solved` 但 `cmd v=0`，任务卡在 RECOVERING）。自由模式之所以没暴露：那条路径的首两点（机器人起点 → 目标）是分开的 | 切线/曲率都改用 `min_span = 5 cm` 基线：点 i 的朝向用"离 i 前后各至少 5 cm 的第一个点"定；κ 同理。这条修好后 `e_yaw0 = −6.7°`、`curv = 0.000`、`cmd v` 正常爬升 |

**验收口径本身的 5 个陷阱**（不是控制器的问题，但不修就会把好好的控制器判死）

| # | 陷阱 | 症状 | 修法 |
|---|---|---|---|
| 1 | **到点容差 = 验收阈值** | `local.goal_tolerance` 原来也是 0.15 ⇒ 管理器在 `remaining ≤ 0.15` 就停机，量到的"到点误差"是**它自己的触发条件**，不是收敛精度（一次 0.141 m，看着合格） | 容差收紧到 0.08（严格小于验收 0.15）⇒ 残差 0.017~0.043 m，这才叫证据 |
| 2 | **候选目标"几何够空" ≠ 全局可达** | 测试挑了一条净距 0.82 m 的直线目标，A* 仍报 `目标不可达` ⇒ 被误判成控制器坏了 | 测试准备**多个候选**，收到 `FAILED` 就换下一个（几何可达性是全局规划器的职责，别的用例已覆盖） |
| 3 | **全程偏差把"起步对线的外摆"算进去了** | 车起步时机头与通道方向差几度，切进线上必然外摆 ⇒ 全程偏差 0.056 > 0.05 判 FAIL，而稳态只有 0.019 | 稳态用硬界（0.05，= `corridor_min_tolerance`），全程留一次外摆（0.10），并在文档里写清两者语义不同（走廊约束的是**参考窗口内**的横向） |
| 4 | **latched 话题给的是上一轮的数据** | `local_status`/`global_path` 都是 latched/持久 ⇒ 一订阅就拿到上次任务的残留，横向误差算出 12.9 m | `Probe.begin()` 在发目标前清空缓存；指标一律用**高频位姿轨迹**自己算 |
| 5 | **`signed_offset` 的索引取错** | 轨迹行是 `(t, x, y, yaw)`，当成 `(x, y, …)` 用 ⇒ 横向误差 16 m，而且数字看着还挺"像真的" | 函数里显式 `x, y = pose[1], pose[2]`，并在 docstring 里写明踩过 |

**已知弱点（下一轮跟）**

- **航向偏差大到 ~40° 以上时，严格走廊下 MPC 会拒动**（也不原地转）：一加速就因航向
  差产生横向偏移、撞走廊硬界，代价上"不动"更便宜。正确行为应是"先原地转正再走"，
  而差速车转正在物理上完全可行（ω 独立于 v）。实测数据（单测里扫过）：航向差
  7/15/25° → 用满加速能力起步；40/60° → `cmd v=0`。已用
  `StrictCorridorWithHeadingOffsetStillDrives` 把现状钉住。
- 走廊前置可达性检查（`computeCommand`）里用 `0.5·v·ω_max·T²` 估计"本时域能纠正
  多少横向偏差"——这是个粗略上界，真实可达集受航向动力学耦合影响，偏大时会把
  "其实能救"的情形误判成 BLOCKED。当前只在偏差明显过大时触发（实测未误触发）。

**试过但否决的方案**（记录下来免得回头再试）

- **放松障碍硬下界为 `min(hard, 参考净距)`**：为了绕开"参考自身在硬距离内导致不可行"。
  单测当场抓到危害 —— 障碍正好压在规划线上时下界退化成 ~0，机器人会**沿规划直插障碍**
  （状态变成 FOLLOWING）。语义必须严格：路径在障碍上就报 BLOCKED。
  失败报错里加了 `refClearanceNote()`，直接指出"参考自身就在障碍硬距离内 → 问题在
  规划/地图不同源"，而不是让人去猜控制器。
- **把 `q_v` 从 3 调大**：曾两次以为"爬行/趴窝"是速度项太弱（第一次算过 3.4 < 10.8，
  第二次算过严格走廊下横向代价 18 > 速度代价 5.5）。但**扫过 q_v=3 与 50**：两种取值
  在所有航向偏差下行为完全一致 ⇒ **不是 q_v 的问题**，真因见上表 #2（速度反馈）
  与 #5（切线基线）。已改回 3。教训：**"算出来像"不等于证据**，要扫参数看行为是否真的变。
- **对 twist 做低通滤波**：滤得掉噪声、滤不掉**均值偏置**，所以没用；换成位姿差分。

**底盘（Go2 仿真）实测特性**（`config/go2_run.yaml` 里记录了完整表格）

- `cmd_vel.linear.x` 经 `cmd_vel_pub.py` 当"步幅参数"用（`0.035·(1−e^{−3.5x})`），
  **静态饱和 ≈0.42 m/s**、`ω` 增益 ≈0.70 ⇒ `v_max=0.42 / w_max=0.65 / reference_speed=0.35`
- 瞬态能冲到 0.89 m/s（> 静态饱和）⇒ 步态控制器**有内部状态**，不是无记忆映射
- lightning 的 `twist.angular.z` **恒为 0**（只有 linear.x 名义上是真速度）
- ⚠ 根治办法在**底盘侧**（让 `cmd_vel_pub` 按 m/s 直通，或标定成 1.0 = 车体最大速度），
  规划器里不写补偿（换底盘就静默调错）—— 这是用户 2026-09-22 明确指出的边界

**E2E 里踩过的坑（测试自身的，值得单列）**

- **latched 话题会给旧数据**：`/pnc_2d/global_path` 是 latched，订阅瞬间收到**上一轮任务**
  残留的路径 ⇒ 用它算横向误差会得出 12.9 m 这种离谱值。必须在给目标前清掉。
- **别用 `ros2 topic echo` 轮询做计时**：每次调用 0.5~1.5 s 启动开销，"sleep 1 s" 实际
  间隔可能 3~4 s（我据此算出过 0.46 m/s 的假速度）。要进程内定时器采样。
- **twist 不能当指标**：末速用**位姿差分**算；速度保真度用"路程 / 指令积分"。
- **目标朝向要合理**：随手写 `orientation.w=1`（yaw=0）而路径方向是 140° 时，
  全局路径"末点取目标朝向"会造出一个**假转弯**（实测 curv 0.49 1/m = 半径 2 m），
  曲率前瞻从 2 m 外开始压速。目标朝向应与行驶方向一致。
- **目标别放在正后方**：首点保留当前朝向 + 末点目标朝向 ⇒ 差 180° 时车只能绕大 U 形
  （走了 5.18 m / 路径 3.75 m），那种弧线不该拿来当"跟踪误差"的验收场景。
- **净距要用发布的全局图**（含禁行区），不能用裸 PGM：否则会选到禁行区里当目标。
- 测试要**自诊断"起点被挡"**这种情形，否则只会报"没有局部状态"，极易误判成控制器坏了。
验收（gazebo + lightning 定位）：
- A→B 全程无碰撞、到点停车（≤ 0.15 m）、**横向误差 < 10 cm**；
- **路网模式**：严格贴线（`corridor_width=0`，偏差 < 5 cm）且**走廊不可违反**；走廊被占 ⇒ `BLOCKED` 停车（不发速度）；
- 自由模式：全局路径 + 障碍软代价，能绕开临时障碍；
- 回归：全局侧 37 单测 + 42 项 E2E 不回归（`local.type: none` 时行为与现在完全一致）。

---

## 6. 风险与对策

| 风险 | 对策 |
|---|---|
| OSQP 0.6.2 是 C API（CSC 矩阵手填繁琐） | 只在 `mpc_local_planner.cpp` 里封装一次；用 Eigen 生 CSC（本包已有 Eigen3 可用） |
| `esdf_2d` 抽点 0.20 m ⇒ 软代价精度有限 | 软代价够用；**硬约束一律走 0.10 m 的 occupancy**（D4）；不够时把 perception 的 `esdf_pub_step` 调成 1（成本在其侧） |
| 走廊硬约束导致频繁停车 | 这是 D5 的明确取舍：安全/可预测优先；靠状态机（P6）做重规划/让行 |
| 局部视野仅 4 m | 时域 1.5 s × 1 m/s = 1.5 m，够用；快速场景再加大 `max_ray_length` |
| 时间同步（ESDF/位姿/地图） | 缓存 + `esdf_timeout` 降级；不允许用过期场做硬约束 |
| CG/CPU 预算 | 热启动 + 固定 QP 结构（只需更新数值）；P99 < 20 ms 作为硬指标 |

---

## 7. 未来优化点：动态障碍速度/轨迹估计

**三档目标**：L0 反应式（本期）→ L1 **速度外推**（性价比最高）→ L2 跟踪 + 短时预测 → L3 学习型/互动式。

| 方案 | 输入 | 做法 | 评价 |
|---|---|---|---|
| **(b) 点云聚类 + CV-KF** ★首选 | `/cloud`（已有） | `pcl::EuclideanClusterExtraction` + 4 状态恒速卡尔曼（最近邻/匈牙利关联） | 30~80 行；给稳定 ID + 速度 + 协方差；遮挡时 predict-only |
| (a) 占据图帧间差分 + 配对 | `grid_map/occupancy_2d` | 差分找新占据 → 聚类 → 上帧配对 | 零新依赖；只有平移速度，噪声大 |
| (c) 成熟跟踪包 | 点云 | `lidar_object_tracking` / Autoware `multi_object_tracker`（EKF+IMM） | 功能全，依赖重、消息需对齐 |
| (d) 占据流/相位相关 | 局部占据图 | 金字塔光流/相位相关 | 最省；多障碍混在一起失效 |
| (e) 高阶 | — | IMM 多模型、VO/RVO/ORCA 相对速度约束、joint-MPC、学习型预测 | 研究性质 |

**接口先预留**（P3/P4 就留好，避免以后大改）：
- 消息 `pnc_2d/DynamicObstacle {id, pose, twist, radius, confidence, stamp}` + 话题 `/pnc_2d/dynamic_obstacles`；
- `LocalPlanner::setDynamicObstacles(...)`（v1 收而不用）；
- MPC 代价里加"**预测位置**处的距离惩罚"：预测器先写"恒速外推"，将来换轨迹预测**只替换预测器**，求解器不动；
- 状态机：`Blocked` 允许由"未来 N 秒走廊将被侵入"触发（**早停**），而不是"已经贴上"；
- 架构：跟踪器放 **perception 侧**（它拥有点云/深度），`pnc_2d` 只消费结果（保持库 ROS-free + 薄壳）。

**工程坑**：时间同步（端到端延迟 >100 ms 基本白做）；遮挡要 predict-only；`max_ray_length=4 m` 限制预判窗口；**行为可预测比预测精度更重要**（明确减速/让行）。

---

## 8. 与参考工程的关系

- **骨架**：ROS 1 `nav_core`（三类）+ `move_base`（状态机），见 `pnc2d_restructure_plan.md` §2。
- **`SLAM-PNC/PNC`**：作为**踩坑参照**（`pnc2d_dev_plan.md` §5）：footprint 必须进主搜索、别在回调里同步长求解、失败别拿旧路径充数、`map_version+episode+seq` 隔离、优化结果要后验校验。它的 `controller`（pure_pursuit + MPC + MINCO）**不照搬**：我们有**人造路网 + 走廊半宽**，贴线可写成硬约束，不需要靠优化猜"该走哪"。

---

## 9. 实测记录（P5 执行时填）

| 项 | 目标 | 实测 |
|---|---|---|
| OSQP 工具链 | 能编译求解 | ✅ 2026-09-22 `solved x=[0.4999,0.4999] iter=25 solve=0.018 ms`（OSQP 0.6.2） |
| 直线跟踪 SSE | < 0.05 m | 待填 |
| 圆弧横向误差 | < 0.10 m | 待填 |
| 走廊越界次数（1000 组随机） | 0 | 待填 |
| 求解耗时 P99（N=15） | < 20 ms | 待填 |
| 与 `scripts/mpc.py` 差异 | ≤ 5% | 待填 |
| 仿真 A→B 横向误差 | < 10 cm | 待填 |
| 路网模式贴线偏差 | < 5 cm（corridor=0） | 待填 |
