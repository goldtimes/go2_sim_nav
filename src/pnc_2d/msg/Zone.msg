# 地图区域层的一个区域（禁行区 / 限速区）。
#
# 来源：map_server 从 routes.yaml 的 `zones:` 段解析（pnc_2d::ZoneSet），通过
# ZoneArray latched 发布给消费者。
#
# ⚠ 与栅格的关系（重要，别重复传）：
#   · 禁行区**已经**被 map_server 按 inflate 烧进发布的全局图，订阅那张图的模块
#     （全局规划器 / RViz / 将来 nav2）自动遵守；
#   · 但**局部**用的是感知的滑动窗 + ESDF，里面没有区域 ⇒ 必须单独把几何传过去，
#     由局部自己融合（见 pnc_2d/local_planner_node）；
#   · 限速区从来就不进栅格，只能靠这个传。
string name

# "forbidden" 或 "speed_limit"（与 pnc_2d::toString(ZoneType) 一致）
string type

# type=speed_limit 时的速度上限 [m/s]；forbidden 恒为 0
float64 value

# 顶点序列（首尾自动闭合，不重复首点；顺序即多边形边界顺序）
geometry_msgs/Polygon polygon
