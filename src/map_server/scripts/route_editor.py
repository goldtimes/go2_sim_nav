#!/usr/bin/env python3
"""路网画线编辑器（离线，不依赖 ROS）

在 2D 栅格地图上当底图，用鼠标点着画"通道"与"区域"，存成 map_server / pnc_2d
能直接吃的 `routes.yaml`（格式见 ../README.md）。

为什么跟着 map_server：`routes.yaml` 是**站点资产**（与 map.yaml 同目录），加载、
校验、发布都在 map_server；画线器是这份资产的编辑工具，理应跟在一起。

为什么不用 RViz 录制：地图本身是运维资产，画线是"离线批改"，用 matplotlib
更省事 —— 底图、坐标、碰撞校验、撤销都在一个窗口里，改完存盘即可。

用法（在 r41_ws 根目录）：
    .venv/bin/python3 src/map_server/scripts/route_editor.py                 # 默认站点
    .venv/bin/python3 src/map_server/scripts/route_editor.py --map-dir /home/gmd/rcs/maps/office4f
    .venv/bin/python3 src/map_server/scripts/route_editor.py --help

交互（窗口里按 h 也会打印一次）：
    —— 通道（lane）——
    左键        在当前通道末尾加一个点（靠近已有节点会自动吸附，用来接线）
    右键        结束当前通道
    n          开始新通道        u  撤销上一步（先撤点，再撤整条通道）
    d          删最后一条通道     f  对最后一条通道做平滑
    a          切换最后一条通道的单/双向
    - / =      最后一条通道限速 -/+ 0.1 m/s
    [ / ]      最后一条通道走廊半宽 -/+ 0.1 m（0 = 严格贴线、遇障即停）
    k          切换最后一条通道"末端节点"的类型（waypoint→station→charge→park）
    —— 区域（zone：禁行 / 限速）——
    z          进入区域模式；左键加顶点、右键闭合多边形（至少 3 个顶点）
    t          切换最后一个区域的类型（forbidden ↔ speed_limit）
    - / =      区域模式下：调最后一个限速区的限速值 -/+ 0.1 m/s
    d / u      区域模式下：删/撤销最后一个区域
    —— 非通道 ——
    s          保存到 <map-dir>/routes.yaml      q  退出

保存前会用**车体矩形**（默认 0.70×0.40 + 0.05 margin）沿每条通道扫一遍，
过不去的通道在图上标红并提示 —— 避免"画得漂亮但车过不去"。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import matplotlib

try:  # 非交互环境（比如 SSH）也能 import 成功，方便单测/校验
    matplotlib.use("TkAgg")
except Exception:  # pragma: no cover
    pass

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

NODE_TYPES = ["waypoint", "station", "charge", "park"]
TYPE_COLOR = {
    "waypoint": "#1f77b4",
    "station": "#2ca02c",
    "charge": "#ff7f0e",
    "park": "#9467bd",
}


# --------------------------------------------------------------------------- 地图
class GridMap:
    """map.yaml + map.pgm → 占据栅格（世界坐标由 origin/resolution 决定）"""

    def __init__(self, map_dir: str) -> None:
        yaml_path = os.path.join(map_dir, "map.yaml")
        if not os.path.isfile(yaml_path):
            raise FileNotFoundError(f"找不到 {yaml_path}")
        with open(yaml_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        pgm = meta.get("image", "map.pgm")
        if not os.path.isabs(pgm):
            pgm = os.path.join(map_dir, pgm)
        self.res = float(meta.get("resolution", 0.05))
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        self.ox, self.oy = float(origin[0]), float(origin[1])
        negate = int(meta.get("negate", 0))
        thr = float(meta.get("occupied_thresh", 0.65))

        self.img = self._read_pgm(pgm)
        h, w = self.img.shape
        val = (255.0 - self.img) / 255.0 if negate == 0 else self.img / 255.0
        # PGM 第 0 行在地图上方 → 翻转，使 occ[i, j] 的 i 沿 +y 增长
        self.occ = np.flipud(val > thr)
        self.w, self.h = w, h
        self.x0, self.y0 = self.ox, self.oy
        self.x1, self.y1 = self.ox + w * self.res, self.oy + h * self.res

    @staticmethod
    def _read_pgm(path: str) -> np.ndarray:
        with open(path, "rb") as f:
            magic = f.readline().strip()
            if magic != b"P5":
                raise ValueError(f"{path} 不是二进制 PGM（P5）")
            line = f.readline()
            while line.startswith(b"#"):
                line = f.readline()
            w, h = (int(v) for v in line.split())
            f.readline()  # maxval
            data = np.frombuffer(f.read(), dtype=np.uint8)
        return data.reshape(h, w)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def cell_occupied(self, i: int, j: int) -> bool:
        if i < 0 or j < 0 or i >= self.h or j >= self.w:
            return True          # 图外当障碍（与 pnc_2d 的语义一致）
        return bool(self.occ[i, j])


def rect_free(grid: GridMap, cx: float, cy: float, yaw: float,
              half_len: float, half_wid: float, step: float = 0.02) -> bool:
    """车体矩形（机体系 x=前后 half_len、y=左右 half_wid）在 (cx,cy,yaw) 处是否自由"""
    c, s = math.cos(yaw), math.sin(yaw)
    for v in np.arange(-half_len, half_len + 1e-9, step):
        for u in np.arange(-half_wid, half_wid + 1e-9, step):
            x = cx + v * c - u * s
            y = cy + v * s + u * c
            i, j = grid.world_to_cell(x, y)
            if grid.cell_occupied(i, j):
                return False
    return True


def lane_free(grid: GridMap, pts: List[Tuple[float, float]], half_len: float,
              half_wid: float, zones: Optional[List[Dict]] = None) -> bool:
    """沿通道逐位姿检查（朝向取该段方向，与 C++ 侧 lineIsConnectionFree 同口径）。

    zones 非空时，**禁行区也算障碍**（按车体外接圆半径膨胀，与 map_server 的
    烧入口径一致）—— 否则编辑器说“通得过”、map_server 说“过不去”，很迷惑。
    """
    inf = math.hypot(half_len, half_wid)
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-9:
            continue
        yaw = math.atan2(by - ay, bx - ax)
        n = max(1, int(math.ceil(seg / max(grid.res / 2.0, 0.02))))
        for k in range(n + 1):
            t = k / n
            cx, cy = ax + (bx - ax) * t, ay + (by - ay) * t
            if not rect_free(grid, cx, cy, yaw, half_len, half_wid):
                return False
            if zones and in_any_forbidden(cx, cy, zones, inf):
                return False
    return True


def in_any_forbidden(x: float, y: float, zones: List[Dict], inflate: float) -> bool:
    """点（或离边界的距离 <= inflate）是否落在任一禁行区内"""
    for z in zones:
        if z["type"] != "forbidden":
            continue
        poly = z["points"]
        if not poly:
            continue
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        if x < min(xs) - inflate or x > max(xs) + inflate \
           or y < min(ys) - inflate or y > max(ys) + inflate:
            continue
        if _point_in_polygon(x, y, poly) or _dist_to_polygon(x, y, poly) <= inflate:
            return True
    return False


def _point_in_polygon(x: float, y: float, poly: List[Tuple[float, float]]) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        xj, yj = poly[i - 1]
        xi, yi = poly[i]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
    return inside


def _dist_to_polygon(x: float, y: float, poly: List[Tuple[float, float]]) -> float:
    best = float("inf")
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 <= 1e-18:
            d = math.hypot(x - ax, y - ay)
        else:
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
            d = math.hypot(x - (ax + dx * t), y - (ay + dy * t))
        best = min(best, d)
    return best


def smooth(points: List[Tuple[float, float]], n: int = 6
           ) -> List[Tuple[float, float]]:
    """Catmull-Rom 样条加密：手点的点少也能得到顺滑通道（端点保持不动）"""
    if len(points) < 3:
        return list(points)
    p = [points[0]] + list(points) + [points[-1]]
    out: List[Tuple[float, float]] = []
    for i in range(1, len(p) - 2):
        p0, p1, p2, p3 = p[i - 1], p[i], p[i + 1], p[i + 2]
        for k in range(n):
            t = k / n
            t2, t3 = t * t, t * t * t
            x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t +
                       (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t +
                       (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
            out.append((x, y))
    out.append(points[-1])
    return out


# --------------------------------------------------------------------------- 编辑器
class RouteEditor:
    def __init__(self, map_dir: str, routes_file: str, lane_len: float,
                 lane_wid: float, margin: float, snap: float) -> None:
        self.grid = GridMap(map_dir)
        self.routes_file = routes_file
        self.half_len = lane_len / 2.0 + margin
        self.half_wid = lane_wid / 2.0 + margin
        self.snap = snap

        self.nodes: Dict[str, Dict] = {}      # name -> {x, y, type}
        self.lanes: List[Dict] = []           # {points, start, end, one_way, speed, corridor}
        self.zones: List[Dict] = []           # {name, type, value, points}  ★区域层
        self.current: List[Tuple[float, float]] = []
        self.zone_current: List[Tuple[float, float]] = []
        self.mode = "lane"                    # lane | zone
        self.cur_start: Optional[str] = None
        self.cur_end: Optional[str] = None
        self.status = "左键加点 / 右键结束通道 / z 画区域 / h 帮助"
        self._new_counter = 0
        self._zone_counter = 0

        if os.path.isfile(routes_file):
            self._load(routes_file)

    # ------------------------------------------------------------------ 载入
    def _load(self, path: str) -> None:
        with open(path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        for n in doc.get("nodes", []):
            self.nodes[str(n["name"])] = {
                "x": float(n["x"]), "y": float(n["y"]),
                "type": str(n.get("type", "waypoint")),
            }
        for e in doc.get("edges", []):
            pts = [(float(p[0]), float(p[1])) for p in e.get("polyline", [])]
            if len(pts) < 2:
                continue
            self.lanes.append({
                "points": pts,
                "start": str(e["from"]),
                "end": str(e["to"]),
                "one_way": bool(e.get("one_way", False)),
                "speed": float(e.get("speed_limit", 1.0)),
                "corridor": float(e.get("corridor_width", 0.5)),
            })
        for z in doc.get("zones", []):
            pts = [(float(p[0]), float(p[1])) for p in z.get("polygon", [])]
            if len(pts) < 3:
                continue
            self.zones.append({"name": str(z.get("name", f"Z{len(self.zones) + 1}")),
                               "type": str(z.get("type", "forbidden")),
                               "value": float(z.get("value", 0.3)),
                               "points": pts})
        print(f"[载入] {path}: {len(self.nodes)} 节点 / {len(self.lanes)} 通道 / "
              f"{len(self.zones)} 区域")

    # ------------------------------------------------------------- 交互
    def run(self) -> None:
        self.fig, self.ax = plt.subplots(figsize=(14, 9))
        self.ax.imshow(self.grid.img, cmap="gray", origin="upper",
                       extent=[self.grid.x0, self.grid.x1, self.grid.y0, self.grid.y1],
                       alpha=0.55, zorder=1)
        self.ax.set_xlim(self.grid.x0, self.grid.x1)
        self.ax.set_ylim(self.grid.y0, self.grid.y1)
        self.ax.set_aspect("equal")
        self.ax.grid(True, lw=0.3, alpha=0.4)
        self.ax.set_title("pnc_2d 路网编辑器 —— 左键加点 / 右键结束 / h 帮助 / s 保存")
        self._print_help()
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.redraw()
        plt.show()

    def _print_help(self) -> None:
        print(__doc__.split("交互（窗口里按 h 也会打印一次）：")[1].split("保存前")[0])

    def _on_click(self, event) -> None:
        if event.inaxes is not self.ax or event.xdata is None:
            return
        x, y = float(event.xdata), float(event.ydata)
        if event.button == 1:            # 左键：加点
            if self.mode == "zone":
                self.zone_current.append((x, y))
                self.status = f"当前区域 {len(self.zone_current)} 个顶点（右键闭合）"
            else:
                name = self._snap_node(x, y)
                if name is not None:
                    x = self.nodes[name]["x"]
                    y = self.nodes[name]["y"]
                if not self.current and name is not None:
                    self.cur_start = name
                self.current.append((x, y))
                if name is not None:
                    if len(self.current) == 1:
                        self.cur_start = name
                    else:
                        self.cur_end = name
                self.status = f"当前通道 {len(self.current)} 点" + (
                    f"，末点吸附到节点 {name}" if name else "")
        elif event.button == 3:          # 右键：结束当前对象
            if self.mode == "zone":
                self._finish_zone()
            else:
                self._finish_lane()
        self.redraw()

    def _on_key(self, event) -> None:
        k = event.key
        if k == "n":
            self._finish_lane()
            self._finish_zone()
            self.mode = "lane"
            self.status = "开始画通道（lane 模式）"
        elif k == "z":
            self._finish_lane()
            self._finish_zone()
            self.mode = "zone"
            self.status = "开始画区域（zone 模式）：左键加顶点、右键闭合、t 切类型"
        elif k == "u":
            if self.mode == "zone" and self.zone_current:
                self.zone_current.pop()
                self.status = "撤销一个顶点"
            elif self.current:
                self.current.pop()
                self.status = "撤销一个点"
            elif self.mode == "zone" and self.zones:
                self.zones.pop()
                self.status = "撤销最后一个区域"
            elif self.lanes:
                self.lanes.pop()
                self.status = "撤销最后一条通道"
        elif k == "d":
            if self.mode == "zone":
                if self.zones:
                    self.zones.pop()
                    self.status = "删除最后一个区域"
            elif self.lanes:
                self.lanes.pop()
                self.status = "删除最后一条通道"
        elif k == "t":
            if self.zones:
                z = self.zones[-1]
                z["type"] = "speed_limit" if z["type"] == "forbidden" else "forbidden"
                self.status = f"区域 {z['name']} 类型 → {z['type']}"
        elif k == "f":
            if self.lanes:
                self.lanes[-1]["points"] = smooth(self.lanes[-1]["points"])
                self.status = "平滑最后一条通道"
        elif k == "a":
            if self.lanes:
                self.lanes[-1]["one_way"] = not self.lanes[-1]["one_way"]
                self.status = f"单向={self.lanes[-1]['one_way']}"
        elif k in ("-", "="):
            dlt = -0.1 if k == "-" else 0.1
            if self.mode == "zone" and self.zones:
                z = self.zones[-1]
                z["value"] = max(0.1, round(z["value"] + dlt, 2))
                self.status = f"区域 {z['name']} 限速={z['value']:.2f} m/s"
            elif self.lanes:
                self.lanes[-1]["speed"] = max(0.1, round(self.lanes[-1]["speed"] + dlt, 2))
                self.status = f"限速={self.lanes[-1]['speed']:.2f} m/s"
        elif k in ("[", "]"):
            if self.lanes:
                dlt = -0.1 if k == "[" else 0.1
                self.lanes[-1]["corridor"] = max(0.0, round(self.lanes[-1]["corridor"] + dlt, 2))
                self.status = (f"走廊半宽={self.lanes[-1]['corridor']:.2f} m"
                               + ("（严格贴线，遇障即停）" if self.lanes[-1]["corridor"] == 0 else ""))
        elif k == "k":
            if self.lanes:
                nm = self.lanes[-1]["end"]
                node = self.nodes.get(nm or "")
                if node:
                    i = (NODE_TYPES.index(node["type"]) + 1) % len(NODE_TYPES)
                    node["type"] = NODE_TYPES[i]
                    self.status = f"节点 {nm} 类型 → {node['type']}"
        elif k == "s":
            self._finish_lane()
            self._finish_zone()
            self.save()
        elif k == "h":
            self._print_help()
        elif k == "q":
            plt.close(self.fig)
        self.redraw()

    # ------------------------------------------------------------- 节点/通道
    def _new_node_name(self) -> str:
        while True:
            self._new_counter += 1
            name = f"P{self._new_counter}"
            if name not in self.nodes:
                return name

    def _snap_node(self, x: float, y: float) -> Optional[str]:
        best, best_d = None, self.snap
        for name, n in self.nodes.items():
            d = math.hypot(n["x"] - x, n["y"] - y)
            if d <= best_d:
                best, best_d = name, d
        return best

    def _finish_lane(self) -> None:
        if len(self.current) < 2:
            if self.current:
                self.status = "通道至少 2 个点，已丢弃"
            self.current, self.cur_start, self.cur_end = [], None, None
            return
        # 端点节点：① 点击时已吸附的名字优先；② 否则看该位置附近有没有已有节点
        # （**必须按坐标去重**：两条通道共用一个交点时不能生成两个同位置节点，
        #   否则路网会在那里断开 —— 自测踩过）
        first, last = self.current[0], self.current[-1]
        start = self.cur_start or self._snap_node(*first) or self._new_node_name()
        self.nodes.setdefault(start, {"x": first[0], "y": first[1], "type": "waypoint"})
        end = self.cur_end or self._snap_node(*last) or self._new_node_name()
        self.nodes.setdefault(end, {"x": last[0], "y": last[1], "type": "waypoint"})
        if start == end:
            self.status = "起止节点相同（自环），已丢弃"
            self.current, self.cur_start, self.cur_end = [], None, None
            return
        # 端点对齐到节点坐标（与 C++ 侧的"自动吸附"一致）
        pts = list(self.current)
        pts[0] = (self.nodes[start]["x"], self.nodes[start]["y"])
        pts[-1] = (self.nodes[end]["x"], self.nodes[end]["y"])
        self.lanes.append({"points": pts, "start": start, "end": end,
                           "one_way": False, "speed": 1.0, "corridor": 0.6})
        self.status = f"完成通道 {start}→{end}（{len(pts)} 点）"
        self.current, self.cur_start, self.cur_end = [], None, None

    def _finish_zone(self) -> None:
        if len(self.zone_current) < 3:
            if self.zone_current:
                self.status = "区域至少 3 个顶点，已丢弃"
            self.zone_current = []
            return
        while True:
            self._zone_counter += 1
            name = f"Z{self._zone_counter}"
            if all(z["name"] != name for z in self.zones):
                break
        self.zones.append({"name": name, "type": "forbidden", "value": 0.3,
                           "points": list(self.zone_current)})
        self.status = f"完成区域 {name}（{len(self.zone_current)} 顶点，默认禁行，t 可切类型）"
        self.zone_current = []

    # ------------------------------------------------------------- 绘制
    def redraw(self) -> None:
        for art in list(self.ax.lines) + list(self.ax.patches) + list(self.ax.texts):
            art.remove()
        bad = 0
        for lane in self.lanes:
            ok = lane_free(self.grid, lane["points"], self.half_len, self.half_wid,
                           self.zones)
            if not ok:
                bad += 1
            xs = [p[0] for p in lane["points"]]
            ys = [p[1] for p in lane["points"]]
            self.ax.plot(xs, ys, "-", lw=2.2, zorder=3,
                         color=("#d62728" if not ok else ("#ff9900" if lane["one_way"] else "#00b0f0")))
            # 方向箭头
            mx, my = xs[len(xs) // 2], ys[len(ys) // 2]
            dx, dy = xs[-1] - xs[0], ys[-1] - ys[0]
            L = math.hypot(dx, dy) or 1.0
            self.ax.annotate("", xy=(mx + dx / L * 0.5, my + dy / L * 0.5),
                             xytext=(mx, my), zorder=4,
                             arrowprops=dict(arrowstyle="-|>", color="#333333", lw=1.4))
        for name, n in self.nodes.items():
            self.ax.plot([n["x"]], [n["y"]], "o", ms=7, zorder=5,
                         color=TYPE_COLOR.get(n["type"], "#1f77b4"))
            self.ax.text(n["x"] + 0.15, n["y"] + 0.15, name, fontsize=8, zorder=6)
        if self.current:
            xs = [p[0] for p in self.current]
            ys = [p[1] for p in self.current]
            self.ax.plot(xs, ys, "-", lw=2.0, color="#00aa00", zorder=3)
            self.ax.plot(xs, ys, ".", ms=6, color="#00aa00", zorder=4)
        for z in self.zones:      # 区域：禁行红、限速橙
            col = "#d62728" if z["type"] == "forbidden" else "#ff8800"
            xs = [p[0] for p in z["points"]]
            ys = [p[1] for p in z["points"]]
            self.ax.fill(xs, ys, color=col, alpha=0.18, zorder=2)
            self.ax.plot(xs + [xs[0]], ys + [ys[0]], "-", lw=2.0, color=col, zorder=3)
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            label = z["name"] if z["type"] == "forbidden" else f"{z['name']} {z['value']:.2f}m/s"
            self.ax.text(cx, cy, label, fontsize=8, ha="center", color=col, zorder=7)
        if self.zone_current:
            xs = [p[0] for p in self.zone_current]
            ys = [p[1] for p in self.zone_current]
            self.ax.plot(xs, ys, "o-", lw=2.0, color="#aa00aa", zorder=3)
        self.ax.set_title(
            f"pnc_2d 路网编辑器 [{self.mode}] | {len(self.lanes)} 通道 / {len(self.nodes)} 节点"
            + (f" / {len(self.zones)} 区域" if self.zones else "")
            + (f" | ⚠ {bad} 条车体过不去（红）" if bad else "")
            + f" | {self.status}")
        self.fig.canvas.draw_idle()

    def _components(self) -> List[List[str]]:
        """按通道的连通性分组（无向）：路网断开是最隐蔽的错误，保存前必须报出来"""
        parent: Dict[str, str] = {n: n for n in self.nodes}

        def find(a: str) -> str:
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        for lane in self.lanes:
            ra, rb = find(lane["start"]), find(lane["end"])
            if ra != rb:
                parent[ra] = rb
        groups: Dict[str, List[str]] = {}
        for n in self.nodes:
            groups.setdefault(find(n), []).append(n)
        return list(groups.values())

    # ------------------------------------------------------------- 保存
    def save(self) -> None:
        bad = []
        for lane in self.lanes:
            if not lane_free(self.grid, lane["points"], self.half_len, self.half_wid,
                             self.zones):
                bad.append(f"{lane['start']}→{lane['end']}")

        lines: List[str] = []
        lines.append("# pnc_2d 路网与区域（由 map_server/scripts/route_editor.py 生成）")
        lines.append("# 通道默认**双向**；需要单向时给该条边加 one_way: true")
        lines.append("# corridor_width: 允许横向偏离的半宽 [m]；0.0 = 严格贴线，遇障即停")
        lines.append("frame_id: map")
        lines.append("nodes:")
        for name, n in self.nodes.items():
            lines.append(f"  - {{name: {name}, x: {n['x']:.3f}, y: {n['y']:.3f}, "
                         f"type: {n['type']}}}")
        lines.append("edges:")
        for lane in self.lanes:
            pts = ", ".join(f"[{x:.3f}, {y:.3f}]" for x, y in lane["points"])
            # 双向是默认语义 → 不写 one_way，保持文件干净易读
            one_way = "one_way: true, " if lane["one_way"] else ""
            lines.append(f"  - {{from: {lane['start']}, to: {lane['end']}, "
                         f"{one_way}"
                         f"speed_limit: {lane['speed']:.2f}, "
                         f"corridor_width: {lane['corridor']:.2f}, "
                         f"polyline: [{pts}]}}")
        if self.zones:
            lines.append("# 区域层：forbidden = 全栈禁行（map_server 会烧进全局图）；"
                         "speed_limit = 供速度规划查询")
            lines.append("zones:")
            for z in self.zones:
                pts = ", ".join(f"[{x:.3f}, {y:.3f}]" for x, y in z["points"])
                extra = f"value: {z['value']:.2f}, " if z["type"] == "speed_limit" else ""
                lines.append(f"  - {{name: {z['name']}, type: {z['type']}, {extra}"
                             f"polygon: [{pts}]}}")
        with open(self.routes_file, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

        total = sum(sum(math.dist(a, b) for a, b in zip(l["points"][:-1], l["points"][1:]))
                    for l in self.lanes)
        print(f"[保存] {self.routes_file}: {len(self.nodes)} 节点 / {len(self.lanes)} 通道 "
              f"/ {len(self.zones)} 区域 / 总长 {total:.1f} m")
        if self.zones:
            forb = sum(1 for z in self.zones if z["type"] == "forbidden")
            spd = len(self.zones) - forb
            print(f"[区域] 禁行 {forb} 个（map_server 会按车体外接圆半径膨胀后烧进全局图）；"
                  f"限速 {spd} 个（不烧入，供速度规划查询）")
        # 重复通道（同起止点）：多半是重复画了一遍，会让图里出现平行边
        seen: Dict[Tuple[str, str], int] = {}
        for lane in self.lanes:
            key = (lane["start"], lane["end"])
            seen[key] = seen.get(key, 0) + 1
        dup = [f"{a}→{b}×{c}" for (a, b), c in seen.items() if c > 1]
        if dup:
            print(f"[警告] 存在重复通道：{', '.join(dup)}（是否多画了一遍？）")
        if bad:
            print(f"[警告] 以下通道车体过不去（已在图上标红）：{', '.join(bad)}")
            print("       原因可能是墙上、也可能是被禁行区挡住了；请挪开通道或收紧区域。")
        comps = self._components()
        if len(comps) > 1:
            print(f"[警告] 路网被分成 {len(comps)} 块（不连通），规划器只能在同一块里找通路：")
            for i, comp in enumerate(comps, 1):
                print(f"       第 {i} 块：{', '.join(comp)}")
            print("       两块之间需要一条通道连起来（端点靠近已有节点会自动吸附）。")
        self.status = (f"已保存（{len(self.lanes)} 通道）"
                       + (f"，⚠{len(bad)} 条不通" if bad else "")
                       + (f"，⚠{len(comps)} 块不连通" if len(comps) > 1 else ""))


# --------------------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description="pnc_2d 路网画线编辑器")
    ap.add_argument("--map-dir", default="/home/gmd/rcs/maps/go2_sim_factory",
                    help="站点地图目录（含 map.yaml + map.pgm）")
    ap.add_argument("--routes", default="", help="路网文件路径（默认 <map-dir>/routes.yaml）")
    ap.add_argument("--lane-length", type=float, default=0.70, help="车体长 [m]")
    ap.add_argument("--lane-width", type=float, default=0.40, help="车体宽 [m]")
    ap.add_argument("--margin", type=float, default=0.05, help="车体四周安全余量 [m]")
    ap.add_argument("--snap", type=float, default=0.30,
                    help="点到已有节点的吸附半径 [m]（用来把通道接起来）")
    args = ap.parse_args()

    routes = args.routes or os.path.join(args.map_dir, "routes.yaml")
    editor = RouteEditor(args.map_dir, routes, args.lane_length, args.lane_width,
                         args.margin, args.snap)
    print(f"[地图] {args.map_dir} | {editor.grid.w}x{editor.grid.h} @ {editor.grid.res} m "
          f"| 范围 x[{editor.grid.x0:.2f},{editor.grid.x1:.2f}] "
          f"y[{editor.grid.y0:.2f},{editor.grid.y1:.2f}]")
    print(f"[输出] {routes}")
    editor.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
