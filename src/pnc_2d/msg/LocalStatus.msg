# 局部规划状态（latched 发布：每个控制周期或状态变化时更新）
#
# 为什么单独一个消息：局部规划的状态语义与全局完全不同 ——
#   "全局规划失败"是"这个目标算不出路"；"局部 BLOCKED"是"路还在、但眼前走不过去"。
#   状态机需要区分这两者才能决定是"重新规划"还是"原地等/跑恢复行为"。
# （对应 core/types.hpp 的 LocalStatus，用独立枚举而不是复用 PlannerStatus，决策 15）
#
# status 取值与 include/pnc_2d/core/types.hpp 的 LocalStatus 一致，
# status_name 是它的字符串形式（见 src/core/types.cpp 的 toString）：
#   0  IDLE          没有可跟随的路径
#   1  FOLLOWING     正在沿路径走
#   2  GOAL_REACHED  到达路径终点
#   3  BLOCKED       前方被挡且不可绕（route 模式即"走廊被占"）→ 停车，交状态机
#   4  DEGRADED      降级运行（例如距离场超时，只剩硬碰撞兜底 + 减速）
#   5  FAILED        求解失败/内部错误

std_msgs/Header header

uint8 status
string status_name      # "FOLLOWING" / "BLOCKED" ...
string message          # 人类可读诊断（失败/降级/被挡的原因）
bool produces_cmd_vel   # 本算法是否会真的发 /pnc_2d/cmd_vel
                        #   false 时（如 NullLocalPlanner）"cmd=0"表示"我不管"，
                        #   而不是"我命令你停下" —— 状态机据此决定要不要转发/仲裁

# 本周期输出（produce_cmd_vel=false 时意义同上，仅作观测）
float64 cmd_v           # 前向速度 [m/s]
float64 cmd_w           # 角速度 [rad/s]

# 诊断（进日志/RViz）
float64 solve_time_ms       # 本周期求解耗时 [ms]
int32 solver_iter           # 求解器迭代数
float64 cross_track_m       # 当前横向偏差 [m]
float64 progress            # 路径进度 0~1
float64 time_to_goal_s      # 预估剩余时间 [s]
uint32 path_points          # 正在跟随的路径点数
bool route_mode             # true = 有走廊约束（贴线走），false = 自由空间跟踪

# ---- 上游剖面一致性（M4.4 / R9：剖面"接通了但没生效"的唯一看得见的证据）----
# 局部每控制周期量"上游说该跑多快 vs 车实际多快"。
# ★ 为什么要进话题：这类故障（use_upstream_profile 被关、q_v 权重为 0、
#   底盘/限速区把它盖住）**没有任何报错**，只表现为"车忽快忽慢"。
float64 profile_dev         # |v_ref − v剖面(s)| / max(v剖面(s), 0.1)；**< 0 = 本周期无剖面**
float64 profile_v           # 上游 v剖面(s) [m/s]
float64 profile_v_ref       # MPC 采纳的参考速度 v_ref [m/s]
float64 profile_track_dev   # |v实测 − v_ref| / max(v_ref, 0.1)；< 0 = 无剖面
