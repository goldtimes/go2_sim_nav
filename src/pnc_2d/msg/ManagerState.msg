# 状态机状态（latched 发布：每次状态转移时更新）
#
# 监控/RViz/"为什么车不动"这类问题的第一现场。终端里：
#   ros2 topic echo /pnc_2d/state
#
# 状态集与 include/pnc_2d/sm/states.hpp 的 State 一致，state_name 是它的字符串形式：
#   0  IDLE         空闲（没有任务）
#   1  PLANNING     正在请求全局规划
#   2  FOLLOWING    跟随路径中
#   3  GOAL_REACHED 到达目标
#   4  RECOVERING   触发恢复行为
#   5  FAILED       本次任务失败（等新目标/Cancel 才复位）

std_msgs/Header header

uint8 state
string state_name
string message          # 转移原因 / 失败原因（人类可读）

# 当前任务（便于日志对齐，不参与逻辑）
float64 goal_x
float64 goal_y

# 当前路径概况
uint32 path_points
float64 path_length_m

# 运行统计（诊断"到底重规划了几次"）
int32 plan_requests     # 已发出的全局规划请求次数
int32 plan_failures     # 其中失败次数
int32 recoveries        # 已触发的恢复行为次数
