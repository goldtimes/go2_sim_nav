#!/usr/bin/env bash
# 轨迹优化离线 A/B 探针（M4.2 起）
#
# 用途：**不启动仿真、不启动 MPC**，只起 map_server + global_planner_node，
#       对**同一张图 + 同一个目标**分别用 `traj.type=none|minco` 规划一次，
#       把两次的 `[traj]` 日志与 service 响应并排打出来。
#
# 为什么必须有它：MINCO 接进主链路之后，"优化到底改了多少 / 值不值得 / 会不会
#   把路走歪"只能靠**同一输入下两次运行的差**来定量（在 RViz 里拿眼睛比两条线
#   是定不了量的）。用户口径：先取证，再改。
#
# 用法：
#   scripts/traj_ab_probe.sh <map.yaml> <sx> <sy> <gx> <gy> [odom_topic]
# 例：
#   scripts/traj_ab_probe.sh data/go2_sim_map/map.yaml -8.15 7.85 -0.15 7.85
set -u

MAP_YAML=${1:?用法: traj_ab_probe.sh <map.yaml> <sx> <sy> <gx> <gy> [odom_topic]}
SX=${2:?}; SY=${3:?}; GX=${4:?}; GY=${5:?}
ODOM_TOPIC=${6:-/lightning/perception/pose}

WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
OUT=/tmp/traj_ab_probe
rm -rf "$OUT"; mkdir -p "$OUT"

export CYCLONEDDS_URI=${CYCLONEDDS_URI:-$WS/src/bringup/cyclonedds.xml}
# ★★ 必须用**与现网不同的域号**（默认 31，不是 30）。
#   为什么：这个脚本会自己起 map_server 往 `global_map/occupancy` 发 latched 图。
#   如果和正在运行的整套系统同域，两件事会同时发生（2026-09-24 实测踩到）：
#     ① 两边的图互相覆盖 —— 后发的那个留在下游订阅者的缓存里；
#     ② 两边都有 `global_planner` 节点，`~/plan_path` 服务名撞在一起，
#        探针调到的可能是**别人的**节点（而别人的节点是旧二进制，字段对不上）。
export ROS_DOMAIN_ID=${PROBE_DOMAIN_ID:-31}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
# ★ `source setup.bash` 必须在 `set -u` **之外**：ROS 的 setup.bash 会引用
#   `COLCON_TRACE` 这个未绑定变量，在 `set -u` 下直接报"未绑定的变量"并**静默**
#   退出（踩过：脚本 0 字节输出、exit 1，原因藏在被重定向掉的 stderr 里）。
set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash" >/dev/null
set -u

for t in none minco; do
  printf '/**:\n  ros__parameters:\n    traj.type: "%s"\n' "$t" > "$OUT/$t.yaml"
done

PIDS=()
cleanup() { for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done; sleep 1; }
trap cleanup EXIT

run_case() {
  local type=$1
  echo "=================== traj.type = $type ==================="
  ros2 launch map_server map_server.launch.py \
      map_yaml:="$MAP_YAML" > "$OUT/map_$type.log" 2>&1 &
  PIDS+=($!)
  sleep 4
  ros2 launch pnc_2d global_planner.launch.py planner_type:=astar \
      extra_config:="$OUT/$type.yaml" > "$OUT/plan_$type.log" 2>&1 &
  PIDS+=($!)
  sleep 5

  # 位姿只发一次即可：节点收到第一条就把 has_odom_ 立起来
  timeout 10 ros2 topic pub --once -w 1 "$ODOM_TOPIC" \
      nav_msgs/msg/Odometry \
      "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $SX, y: $SY}, \
orientation: {w: 1.0}}}, twist: {twist: {linear: {x: 0.0}}}}" \
      > "$OUT/odom_$type.log" 2>&1
  sleep 2

  timeout 60 ros2 service call /global_planner/plan_path \
      pnc_2d/srv/PlanPath \
      "{header: {frame_id: 'map'}, \
start: {header: {frame_id: 'map'}, pose: {position: {x: $SX, y: $SY}, orientation: {w: 1.0}}}, \
goal: {header: {frame_id: 'map'}, pose: {position: {x: $GX, y: $GY}, orientation: {w: 1.0}}}, \
use_current_pose: false, publish_result: false}" \
      > "$OUT/srv_$type.log" 2>&1
  sleep 1

  grep -aE "\[planner\] 全局|\[traj\]|\[planner\] 规划请求" "$OUT/plan_$type.log" \
      | sed 's/^/  /'
  # service 响应很长（含整条 path），只挑关键字段
  echo "  --- 响应 ---"
  python3 - "$OUT/srv_$type.log" <<'PY'
import re
import sys

txt = open(sys.argv[1], errors='replace').read()
tail = txt.split('response:', 1)[-1]   # 只要响应部分，避免匹配到请求里的字段


def field(name):
    m = re.search(name + r"=([^,)]*)", tail)
    return m.group(1).strip() if m else '?'


def arr_len(name):
    m = re.search(name + r"=\[(.*?)\](?:,|\))", tail, re.S)
    if not m:
        return 0
    body = m.group(1).strip()
    return 0 if not body else len(body.split(','))


print('  status      %s (%s)' % (field('status'), field('status_name')))
print('  success     %s | plan_time_ms %s' % (field('success'),
                                              field('plan_time_ms')))
print('  path.poses  %d 个' % tail.count('PoseStamped('))
print('  traj_valid  %s | traj_note %s' % (field('traj_valid'),
                                           field('traj_note')))
for k in ('traj_s', 'traj_v', 'traj_w'):
    print('  %-10s %d 个' % (k, arr_len(k)))
PY

  cleanup
  PIDS=()
  sleep 1
}

echo "地图 $MAP_YAML | 起点 ($SX, $SY) → 目标 ($GX, $GY) | 域 $ROS_DOMAIN_ID"
run_case none
run_case minco
echo "原始日志在 $OUT/"
