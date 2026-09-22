#!/usr/bin/env bash
# 跑 pnc_2d 的全部端到端测试（会自动起停隔离的规划节点，不干扰正在跑的系统）。
#
#   bash src/pnc_2d/test/e2e/run_all.sh
#
# 前置：已 colcon build 并 source install/setup.bash；本仓库的 DDS 环境见下。
# 注意：不要用 `set -u` —— ROS 的 setup.bash 会引用未绑定变量（COLCON_TRACE 等）。
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${PNC2D_WS:-$(cd "$HERE/../../../.." && pwd)}"

# 本仓库约定：CycloneDDS + domain 30（与 r41_ws 其它节点一致）
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-$WS/src/bringup/cyclonedds.xml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "找不到 $WS/install/setup.bash：先 colcon build" >&2
  exit 2
fi
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

rc=0
run() {
  echo
  echo "================ $1 ================"
  shift
  python3 "$@" || rc=1
}

run "E2E-1 路径有效期（默认：到达目标不清空）" "$HERE/test_clear_semantics.py"
echo
echo "================ E2E-1b 路径有效期（打开 clear.auto_on_goal_reached）================"
EXPECT_AUTOCLEAR=1 python3 "$HERE/test_clear_semantics.py" || rc=1
run "E2E-2 换算法重启后的清场" "$HERE/test_restart_cleanup.py"
run "E2E-3 运行时热切换 / 热重载" "$HERE/test_hot_switch.py"

echo
if [[ $rc -eq 0 ]]; then
  echo "全部 E2E 通过"
else
  echo "有 E2E 失败（节点日志：/tmp/pnc2d_e2e_*.log）" >&2
fi
exit $rc
