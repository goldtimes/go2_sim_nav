# 全局规划状态（latched 发布：每次规划请求结束后更新一次）
#
# 为什么需要它：失败时 /pnc_2d/global_path 会被**显式清空**（发一条空 Path），
# 上层"只看路径"的节点最多只能知道"现在没有路径"，却不知道为什么。
# 本消息给出机器可读的原因，供状态机 / 上层上报错误使用。
#
# status 取值与 include/pnc_2d/core/types.hpp 的 PlannerStatus 一致，
# status_name 就是它的字符串形式（见 src/core/types.cpp 的 toString）：
#   0  SUCCESS                    1  NOT_INITIALIZED
#   2  INVALID_INPUT              3  START_OUT_OF_MAP
#   4  GOAL_OUT_OF_MAP            5  START_OCCUPIED
#   6  GOAL_OCCUPIED              7  START_FOOTPRINT_COLLISION
#   8  GOAL_FOOTPRINT_COLLISION   9  NO_PATH
#   10 TIMEOUT                    11 MAX_ITERATIONS

std_msgs/Header header

uint8 status            # 见上面的取值表
string status_name      # 枚举名，例如 "SUCCESS" / "NO_PATH"
string message          # 人类可读诊断（失败原因，可能含"哪条通道过不去"）
bool success            # 等价于 status == 0

# 本次请求（便于日志/复现对齐）
float64 start_x
float64 start_y
float64 goal_x
float64 goal_y

# 结果与统计
uint32 path_points      # 路径点数（失败时 0）
float64 path_length_m   # 路径总长 [m]
float64 plan_time_ms    # 本次规划耗时 [ms]
int32 expanded_nodes    # 搜索扩展节点数
int32 windows_tried     # 搜索窗口重试次数
