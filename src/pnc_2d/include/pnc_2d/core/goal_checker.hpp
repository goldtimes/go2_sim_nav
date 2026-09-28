// 到点判定（M5.1）：把"到没到"这一件事收拢到**一个**可单测的地方。
//
// == 为什么需要它 ==
// 原来"到点"散在两处、各写一遍：
//   · 节点层 `LocalPlannerNode::finishReached()` —— 沿向（扣停车惯性）+ 横向 +
//   朝向，
//     三条都过才算到达；`describeResidual()` 负责"哪一个没过"；
//   · 算法层 `MpcLocalPlanner::goalYawAlign()` ——
//   到终点附近朝向不对就**原地对正**。
// 两处**共用同一个 yaw 容差**（刻意如此：各配一份会出现"算法按 5° 对正、节点按
// 2° 判"⇒ 永远不满足 ⇒ 任务卡到超时）。但口径、报数、锁存各一份 ⇒ 无法单测、
// 也说不清谁在管。
//
// == 语义（与 Nav2 `SimpleGoalChecker` 同构，但口径更适合本项目）==
//   · **不判"到目标点的欧氏距离"**：本项目判的是**沿路径的两个正交分量**
//     （`along` 还剩多少 / `lateral` 偏了多少）。理由见节点里 `EndResidual`
//     的注释：
//     同一个欧氏数会把"没走到"和"停在侧面"混成一个数，实测让两边结论相反。
//   · `stop_coast`：底盘指令归零后还会自己走一段（实测 ~9 cm）⇒
//   判据要**扣掉它**，
//     让指令提前归零、惯性正好把最后几厘米带完（这是"到点 ≤3
//     cm"能成立的关键）。
//   · `stateful`（Nav2 同名语义）：**一旦满足就锁住**，之后恒 true 直到
//   `reset()`。
//     它治的是一个真实故障：算法已报"到终点"，但**判定之后车还会滑行 ~9 cm**，
//     残差可能被推出容差 ⇒ 现在的实现会把一次成功的到点报成
//     `FAILED`（假失败）。 锁存同时避免"到点→漂走→再判到点"重复上报。
//
// == 谁配什么 ==
//   容差全部由**调用方**（节点）每周期灌进来：`local.goal_tolerance` /
//   `local.lateral_tolerance`（节点参数）+ `goalYawTolerance()` / `stopCoast()`
//   （**算法**参数）。这样"算法怎么对正"与"节点怎么判"仍然是**同一份**数据，
//   而"怎么判"只有这一个实现。

#pragma once

#include <string>

namespace pnc_2d {

/// 到点残差（三个量都要**原始值**，本类负责扣 `stop_coast`）
struct GoalResidual {
  double along{0.0};   ///< 沿路径剩余 [m]（= 沿向残差，已含停车惯性）
  double lateral{0.0}; ///< 横向残差 [m]（到路径末段的垂距）
  double yaw_err{0.0}; ///< 目标朝向误差 [rad]（**绝对值**，调用方先 wrap）
  bool has_yaw{false}; ///< 路径末点是否带朝向；false = 本任务不要求朝向
};

class GoalChecker {
public:
  struct Params {
    double along_tolerance{0.01};   ///< 沿向容差 [m]
    double lateral_tolerance{0.10}; ///< 横向容差 [m]（与沿向正交，单独判）
    double yaw_tolerance{0.0};      ///< 朝向容差 [rad]；<=0 = 不判朝向
    double stop_coast{0.0};         ///< 停车惯性 [m]：沿向判据里扣掉的量
    bool stateful{true};            ///< true = 满足后锁存
  };

  /// 灌参数（**不动锁存**：参数热重载不能把"已到"抖掉）
  void configure(const Params &p) { p_ = p; }
  const Params &params() const { return p_; }

  /// 新任务开始时调用（清锁存与上升沿）
  void reset() {
    latched_ = false;
    prev_ok_ = false;
    just_latched_ = false;
  }

  /// 纯判定（不碰任何状态）：三条都满足才 true
  bool satisfied(const GoalResidual &r) const;

  /// 带锁存的主入口。stateful 时一旦满足即锁存，之后恒 true（直到 reset）。
  bool update(const GoalResidual &r);

  bool latched() const { return latched_; }
  /// 本周期是否**刚刚**满足（上升沿）—— 只打一次日志/只上报一次事件用
  bool justLatched() const { return just_latched_; }

  /// 哪一条没过："" / "along" / "lateral" / "yaw"（三条按优先级）
  const char *firstFailure(const GoalResidual &r) const;

  /// 人读的诊断串（未到点/失败上报必须说清"差在哪、差多少、容差多少"；
  /// 只报一句"没到"没有任何信息量 —— 这是本项目反复踩过的）
  std::string describe(const GoalResidual &r) const;

private:
  Params p_{};
  bool latched_{false};
  bool prev_ok_{false};
  bool just_latched_{false};
};

} // namespace pnc_2d
