#!/usr/bin/env python3
"""Route network editor (offline, no ROS).

Draw lanes and zones on top of a 2D occupancy map and save them as `routes.yaml`
(consumed by map_server / pnc_2d; format documented in ../README.md).

Why it lives with map_server: `routes.yaml` is a *station asset* (same directory
as map.yaml); loading, validating and publishing it is map_server's job, so the
authoring tool lives here too.

Usage (from the r41_ws root):
    .venv/bin/python3 src/map_server/scripts/route_editor.py                 # default station
    .venv/bin/python3 src/map_server/scripts/route_editor.py --map-dir /home/gmd/rcs/maps/office4f
    .venv/bin/python3 src/map_server/scripts/route_editor.py --help

NOTE: all window / terminal text is English on purpose -- matplotlib cannot render
CJK without a CJK font installed (Chinese labels showed up as boxes).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import matplotlib

try:  # 非交互环境（SSH/CI）也能 import 成功，方便脚本化自测
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

HELP = """\
Interaction (also printed by pressing `h` in the window):

  --- lane mode (default) ---
  Every left click becomes a NODE (a named point). Consecutive clicks are
  connected in order, so both "click along and draw" and "place points first,
  connect them afterwards" work the same way.
  left click   append the next point of the current lane. An existing node within
               --snap metres is REUSED (this is how you connect / branch from an
               existing point); otherwise a new node is created right there.
  right click  finish the current lane (needs >= 2 points)
  u            undo: drop last point of the current lane, else drop the last lane.
               A point that this lane created itself is removed with it (unless
               another lane uses it), so no orphan points are left behind
  d            delete last lane
  f            smooth last lane (spline; node positions are kept as vertices)
  v            SPLIT last lane at every point -> one short lane per segment.
               Recommended: map_server judges feasibility per whole edge, so a
               single blocked segment then only kills that one lane (the rest
               still route), and the intermediate points become real junctions
  a            toggle one-way of last lane (default: two-way)
  - / =        speed limit of last lane          -/+ 0.1 m/s
  [ / ]        corridor half width of last lane  -/+ 0.1 m (0 = strict lane
               keeping: stop when blocked)
  k            cycle type of the lane's END node (waypoint/station/charge/park)

  --- delete mode (`x`, or press Delete/Backspace) ---
  left click   delete whatever is under the cursor. Pick order: point > lane >
               zone, and the target is highlighted in red while hovering.
               * point that no lane uses  -> the point is removed
               * point used by lanes     -> the point is removed AND every lane
                 that used it is removed as well
               * lane                    -> the lane is removed, its points stay
               * zone                    -> the zone is removed
  shift+click  same, but afterwards also drop the waypoint(s) that this deletion
               just orphaned. Points you placed on purpose (station / charge /
               park) are never removed automatically.
  x            leave delete mode (back to lane mode)

  --- point mode ---
  left click   place a standalone point (useful for stations / landmarks)
  k            cycle type of the last placed point
  u / d        remove the last placed point (refused while a lane uses it --
               use delete mode `x` if you want to remove it together with that lane)

  --- zone mode (forbidden / speed_limit) ---
  z            enter zone mode
  left click   add a polygon vertex
  right click  close the polygon (>= 3 vertices; new zone is `forbidden`)
  t            toggle type of the last zone (forbidden <-> speed_limit)
  - / =        speed limit of the last speed zone  -/+ 0.1 m/s
  u / d        undo / delete last zone

  --- common ---
  n / p / z    lane mode / point mode / zone mode
  x            delete mode (click a point / lane / zone to remove it)
  s            save to <map-dir>/routes.yaml      h  help      q  quit

matplotlib's own single-key shortcuts (k = log axis, s = save figure, h = home,
f = fullscreen, p = pan, backspace = back) are disabled so they cannot clash.

On save three checks run with the vehicle rectangle (0.70 x 0.40 + 0.05 margin
by default) and problems are printed plus highlighted in the window:
  1. passability of every lane (including inflated forbidden zones, the same rule
     map_server uses when burning zones into the global map);
  2. connectivity -- a split network is the most hidden mistake, each component
     is listed;
  3. duplicate lanes (same from/to pair).
"""


# --------------------------------------------------------------------------- 地图
class GridMap:
    """map.yaml + map.pgm → 占据栅格（世界坐标由 origin/resolution 决定）"""

    def __init__(self, map_dir: str) -> None:
        yaml_path = os.path.join(map_dir, "map.yaml")
        if not os.path.isfile(yaml_path):
            raise FileNotFoundError(f"map.yaml not found: {yaml_path}")
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
                raise ValueError(f"{path} is not a binary PGM (P5)")
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


def disable_default_keymaps() -> List[str]:
    """屏蔽 matplotlib 自带的单键快捷键。

    FigureManagerBase 会自动注册 key_press_handler，而它的默认 keymap 与本画线器
    抢键：k = x 轴对数坐标、s = 存图、h = home、f = 全屏、p = pan、backspace = 后退。
    例如按 k 切节点类型时，x 轴会被切成对数坐标，画面直接废。
    方向键（'left'/'right' 等多字符键）保留，但 backspace 要去掉（本工具用 Delete/
    Backspace 进入删除模式）。
    """
    cleared: List[str] = []
    for key, val in list(matplotlib.rcParams.items()):
        if key.startswith("keymap.") and isinstance(val, (list, tuple)):
            keep = [c for c in val
                    if not (isinstance(c, str) and (len(c) == 1 or c == "backspace"))]
            if len(keep) != len(val):
                matplotlib.rcParams[key] = keep
                cleared.append(key)
    return cleared


def _dist_to_polyline(x: float, y: float, pts: List[Tuple[float, float]]) -> float:
    """点到一段未闭合折线（通道）的最短距离"""
    best = float("inf")
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 <= 1e-18:
            d = math.hypot(x - ax, y - ay)
        else:
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
            d = math.hypot(x - (ax + dx * t), y - (ay + dy * t))
        best = min(best, d)
    return best


def in_any_forbidden(x: float, y: float, zones: List[Dict], inflate: float) -> bool:
    """点（或离边界距离 <= inflate）是否落在任一禁行区内"""
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


def lane_free(grid: GridMap, pts: List[Tuple[float, float]], half_len: float,
              half_wid: float, zones: Optional[List[Dict]] = None,
              zone_inflate: float = 0.0) -> bool:
    """沿通道逐位姿检查（朝向取该段方向，与 C++ 侧 lineIsCollisionFree 同口径）。

    zones 非空时**禁行区也算障碍**，并按 `zone_inflate` 膨胀（<0 = 车体外接圆半径）。
    这个值必须与 map_server 的 `zones.inflate` 一致
    —— 否则编辑器说"通得过"、map_server 说"过不去"，很迷惑。
    """
    inf = zone_inflate if zone_inflate >= 0.0 else math.hypot(half_len, half_wid)
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
                 lane_wid: float, margin: float, snap: float,
                 zone_inflate: float = 0.0) -> None:
        self.grid = GridMap(map_dir)
        self.routes_file = routes_file
        self.half_len = lane_len / 2.0 + margin
        self.half_wid = lane_wid / 2.0 + margin
        self.snap = snap
        # 禁行区膨胀量：必须与 map_server 的 zones.inflate 一致，否则会出现
        # “编辑器说通得过 / map_server 说过不去”。<0 = 自动用车体外接圆半径。
        self.zone_inflate = (zone_inflate if zone_inflate >= 0.0
                             else math.hypot(self.half_len, self.half_wid))

        self.nodes: Dict[str, Dict] = {}      # name -> {x, y, type}
        self.lanes: List[Dict] = []           # {points, start, end, via, one_way, speed, corridor}
        self.zones: List[Dict] = []           # {name, type, value, points}
        self.current: List[Tuple[float, float]] = []
        self.current_nodes: List[str] = []    # 当前通道途经的节点名（顺序）
        self.current_created: List[bool] = []  # 与 current_nodes 平行：该点是否本次新建
        self.zone_current: List[Tuple[float, float]] = []
        self.mode = "lane"                    # lane | point | zone | delete
        self.last_point: Optional[str] = None  # 最近放置的点（point 模式用）
        self.hover: Tuple[Optional[str], Optional[object]] = (None, None)
        self.hover_key: Optional[Tuple] = None  # 悬停对象变化时才重绘
        self.status = "left click: add point / right click: finish lane / h: help"
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

        def node_at(x: float, y: float) -> Optional[str]:
            best, best_d = None, 1e-6
            for nm, nd in self.nodes.items():
                d = math.hypot(nd["x"] - x, nd["y"] - y)
                if d <= best_d:
                    best, best_d = nm, d
            return best

        for e in doc.get("edges", []):
            pts = [(float(p[0]), float(p[1])) for p in e.get("polyline", [])]
            if len(pts) < 2:
                continue
            via = [nm for nm in (node_at(*p) for p in pts) if nm is not None]
            self.lanes.append({
                "points": pts,
                "start": str(e["from"]),
                "end": str(e["to"]),
                "via": via,
                "one_way": bool(e.get("one_way", False)),
                "speed": float(e.get("speed_limit", 1.0)),
                "corridor": float(e.get("corridor_width", 0.6)),
            })
        for z in doc.get("zones", []):
            pts = [(float(p[0]), float(p[1])) for p in z.get("polygon", [])]
            if len(pts) < 3:
                continue
            self.zones.append({"name": str(z.get("name", f"Z{len(self.zones) + 1}")),
                               "type": str(z.get("type", "forbidden")),
                               "value": float(z.get("value", 0.3)),
                               "points": pts})
        print(f"[load] {path}: {len(self.nodes)} nodes / {len(self.lanes)} lanes / "
              f"{len(self.zones)} zones")

    # ------------------------------------------------------------- 交互
    def run(self) -> None:
        print(f"[keys] disabled matplotlib default shortcuts: "
              f"{', '.join(disable_default_keymaps())}")
        self.fig, self.ax = plt.subplots(figsize=(14, 9))
        self.ax.imshow(self.grid.img, cmap="gray", origin="upper",
                       extent=(self.grid.x0, self.grid.x1, self.grid.y0, self.grid.y1),
                       alpha=0.55, zorder=1)
        self.ax.set_xlim(self.grid.x0, self.grid.x1)
        self.ax.set_ylim(self.grid.y0, self.grid.y1)
        self.ax.set_aspect("equal")
        self.ax.grid(True, lw=0.3, alpha=0.4)
        self.ax.set_title("pnc_2d route editor")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self._print_help()
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_move)
        self.redraw()
        plt.show()

    def _print_help(self) -> None:
        print(HELP)

    def _on_click(self, event) -> None:
        if event.inaxes is not self.ax or event.xdata is None:
            return
        x, y = float(event.xdata), float(event.ydata)

        if event.button == 1:                     # left click
            if self.mode == "delete":
                self._delete_at(x, y, cleanup=bool(event.key == "shift"))
            elif self.mode == "zone":
                self.zone_current.append((x, y))
                self.status = f"zone: {len(self.zone_current)} vertices (right click closes)"
            elif self.mode == "point":
                name, created = self._ensure_node(x, y)
                self.last_point = name
                self.status = (f"point {name} placed" if created
                               else f"point {name} already exists (reused)")
            else:                                 # lane
                name, created = self._ensure_node(x, y)
                if name in self.current_nodes:
                    self.status = f"point {name} is already in this lane"
                else:
                    self.current_nodes.append(name)
                    self.current_created.append(created)
                    self.current.append((self.nodes[name]["x"], self.nodes[name]["y"]))
                    self.status = ("lane: " + " -> ".join(self.current_nodes)
                                   + ("  (new node)" if created else "  (existing node)"))
        elif event.button == 3:                   # right click
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
            self.status = "lane mode: click points; existing nodes are reused"
        elif k == "p":
            self._finish_lane()
            self._finish_zone()
            self.mode = "point"
            self.status = "point mode: click to place standalone points (k cycles type)"
        elif k == "z":
            self._finish_lane()
            self._finish_zone()
            self.mode = "zone"
            self.status = "zone mode: left click adds vertices, right click closes"
        elif k in ("x", "delete", "backspace"):
            self._finish_lane()
            self._finish_zone()
            if self.mode == "delete":
                self.mode = "lane"
                self.status = "lane mode: click points; existing nodes are reused"
            else:
                self.mode = "delete"
                self.status = ("delete mode: click a point / lane / zone; "
                               "shift+click also cleans orphaned points")
            self.hover, self.hover_key = (None, None), None
        elif k == "u":
            if self.mode == "zone" and self.zone_current:
                self.zone_current.pop()
                self.status = "removed last zone vertex"
            elif self.zone_current:
                self.zone_current.pop()
                self.status = "removed last zone vertex"
            elif self.mode == "point" and self.last_point:
                self._remove_last_point()
            elif self.current_nodes:
                name = self.current_nodes.pop()
                self.current.pop()
                created = self.current_created.pop() if self.current_created else False
                if created and not self._lanes_using(name):
                    self.nodes.pop(name, None)   # 本次新建且没人引用 → 一并收回
                self.status = ("lane: " + " -> ".join(self.current_nodes)
                               if self.current_nodes else "lane cleared")
            elif self.lanes:
                self.lanes.pop()
                self.status = "removed last lane"
            elif self.zones:
                self.zones.pop()
                self.status = "removed last zone"
        elif k == "d":
            if self.mode == "zone":
                if self.zones:
                    self.zones.pop()
                    self.status = "removed last zone"
            elif self.mode == "point":
                self._remove_last_point()
            elif self.lanes:
                self.lanes.pop()
                self.status = "removed last lane"
        elif k == "t":
            if self.zones:
                z = self.zones[-1]
                z["type"] = "speed_limit" if z["type"] == "forbidden" else "forbidden"
                self.status = f"zone {z['name']} type -> {z['type']}"
        elif k == "f":
            if self.lanes:
                self.lanes[-1]["points"] = smooth(self.lanes[-1]["points"])
                self.status = "smoothed last lane"
        elif k == "v":
            self._split_last_lane()
        elif k == "a":
            if self.lanes:
                self.lanes[-1]["one_way"] = not self.lanes[-1]["one_way"]
                self.status = f"one_way = {self.lanes[-1]['one_way']}"
        elif k in ("-", "="):
            dlt = -0.1 if k == "-" else 0.1
            if self.mode == "zone" and self.zones:
                z = self.zones[-1]
                z["value"] = max(0.1, round(z["value"] + dlt, 2))
                self.status = f"zone {z['name']} speed limit = {z['value']:.2f} m/s"
            elif self.lanes:
                self.lanes[-1]["speed"] = max(0.1, round(self.lanes[-1]["speed"] + dlt, 2))
                self.status = f"lane speed limit = {self.lanes[-1]['speed']:.2f} m/s"
        elif k in ("[", "]"):
            if self.lanes:
                dlt = -0.1 if k == "[" else 0.1
                self.lanes[-1]["corridor"] = max(0.0, round(self.lanes[-1]["corridor"] + dlt, 2))
                self.status = (f"corridor half width = {self.lanes[-1]['corridor']:.2f} m"
                               + ("  (strict lane keeping)" if self.lanes[-1]["corridor"] == 0 else ""))
        elif k == "k":
            name = self.last_point if self.mode == "point" else (
                self.lanes[-1]["end"] if self.lanes else None)
            node = self.nodes.get(name or "")
            if node:
                i = (NODE_TYPES.index(node["type"]) + 1) % len(NODE_TYPES) \
                    if node["type"] in NODE_TYPES else 0
                node["type"] = NODE_TYPES[i]
                self.status = f"node {name} type -> {node['type']}"
        elif k == "s":
            self._finish_lane()
            self._finish_zone()
            self.save()
        elif k == "h":
            self._print_help()
        elif k == "q":
            plt.close(self.fig)
        self.redraw()

    def _on_move(self, event) -> None:
        """删除模式：鼠标悬停对象变红（只在目标变化时重绘，避免刷帧卡顿）"""
        if self.mode != "delete" or event.inaxes is not self.ax or event.xdata is None:
            if self.hover_key is not None:
                self.hover, self.hover_key = (None, None), None
                self.redraw()
            return
        kind, obj = self._pick(float(event.xdata), float(event.ydata))
        key = (kind, id(obj)) if obj is not None else (kind, None)
        if key != self.hover_key:
            self.hover_key, self.hover = key, (kind, obj)
            self.redraw()

    # ------------------------------------------------------------- 删除
    def _pick_radius(self) -> float:
        """拾取半径：鼠标不需要点得很准，但别大到能误删邻点"""
        return max(self.snap, 0.40)

    def _pick(self, x: float, y: float) -> Tuple[Optional[str], Optional[object]]:
        """命中测试，优先级：点 > 通道 > 区域；没命命中返回 (None, None)"""
        r = self._pick_radius()
        best, best_d = None, r
        for name, n in self.nodes.items():
            d = math.hypot(n["x"] - x, n["y"] - y)
            if d <= best_d:
                best, best_d = name, d
        if best is not None:
            return "node", best

        lane_hit, best_d = None, r
        for lane in self.lanes:
            d = _dist_to_polyline(x, y, lane["points"])
            if d <= best_d:
                lane_hit, best_d = lane, d
        if lane_hit is not None:
            return "lane", lane_hit

        for z in self.zones:
            if _point_in_polygon(x, y, z["points"]):
                return "zone", z
        return None, None

    def _drop_from_current(self, name: str) -> None:
        """把一个点从未结束的通道里抽掉（三个平行列表要同步）"""
        while name in self.current_nodes:
            i = self.current_nodes.index(name)
            self.current_nodes.pop(i)
            if i < len(self.current):
                self.current.pop(i)
            if i < len(self.current_created):
                self.current_created.pop(i)

    def _cleanup_orphans(self, candidates: List[str]) -> List[str]:
        """只清理由本次删除直接造成的孤立点，两个保险：
        1) 候选必须是刚被删对象引用过的点（不会动用户特意放的站点）；
        2) 类型必须是 waypoint（station/charge/park 即使孤立也保留）。"""
        orphans = [n for n in dict.fromkeys(candidates)
                   if n in self.nodes
                   and n not in self.current_nodes
                   and self.nodes[n]["type"] == "waypoint"
                   and not self._lanes_using(n)]
        for n in orphans:
            self.nodes.pop(n, None)
            if self.last_point == n:
                self.last_point = None
        if orphans:
            print(f"[delete] cleaned {len(orphans)} orphan point(s): "
                  f"{', '.join(orphans)}")
        return orphans

    def _delete_at(self, x: float, y: float, cleanup: bool = False) -> None:
        kind, obj = self._pick(x, y)
        if kind is None:
            self.status = "delete: nothing under the cursor"
            return
        touched: List[str] = []        # 受影响、可能变孤立的点
        if kind == "node":
            name: str = obj                                        # type: ignore[assignment]
            users = self._lanes_using(name)
            if users:
                gone = ", ".join(f"{l['start']}->{l['end']}" for l in users)
                for l in users:
                    touched += l.get("via") or [l["start"], l["end"]]
                self.lanes = [l for l in self.lanes if l not in users]
                self.nodes.pop(name, None)
                print(f"[delete] point {name} removed, together with "
                      f"{len(users)} lane(s) that used it: {gone}")
                self.status = (f"deleted point {name} "
                               f"(+{len(users)} lane(s): {gone})")
            else:
                self.nodes.pop(name, None)
                print(f"[delete] point {name} removed")
                self.status = f"deleted point {name}"
            if self.last_point == name:
                self.last_point = None
            self._drop_from_current(name)
        elif kind == "lane":
            self.lanes.remove(obj)                                 # type: ignore[arg-type]
            touched = obj.get("via") or [obj["start"], obj["end"]]  # type: ignore[union-attr]
            print(f"[delete] lane {obj['start']}->{obj['end']} removed")  # type: ignore[index]
            self.status = f"deleted lane {obj['start']}->{obj['end']}"    # type: ignore[index]
        else:
            self.zones.remove(obj)                                 # type: ignore[arg-type]
            print(f"[delete] zone {obj['name']} removed")                 # type: ignore[index]
            self.status = f"deleted zone {obj['name']}"                   # type: ignore[index]
        if cleanup:
            n = self._cleanup_orphans(touched)
            if n:
                self.status += f", cleaned {len(n)} orphan(s)"
        self.hover, self.hover_key = (None, None), None

    # ------------------------------------------------------------- 节点/通道
    def _new_node_name(self) -> str:
        while True:
            self._new_counter += 1
            name = f"P{self._new_counter}"
            if name not in self.nodes:
                return name

    def _snap_node(self, x: float, y: float) -> Optional[str]:
        """最近的已有节点（吸附半径内），没有则 None"""
        best, best_d = None, self.snap
        for name, n in self.nodes.items():
            d = math.hypot(n["x"] - x, n["y"] - y)
            if d <= best_d:
                best, best_d = name, d
        return best

    def _ensure_node(self, x: float, y: float) -> Tuple[str, bool]:
        """复用吸附到的已有节点，否则在 (x,y) 新建一个 → (名字, 是否新建)"""
        name = self._snap_node(x, y)
        if name is not None:
            return name, False
        name = self._new_node_name()
        self.nodes[name] = {"x": x, "y": y, "type": "waypoint"}
        return name, True

    def _remove_last_point(self) -> None:
        name = self.last_point
        if name is None:
            self.status = "no point to remove"
            return
        used = any(name in [l["start"], l["end"]] or name in l.get("via", [])
                   for l in self.lanes)
        if used:
            self.status = f"point {name} is used by a lane -- delete that lane first"
            return
        self.nodes.pop(name, None)
        self.last_point = None
        self.status = f"point {name} removed"

    def _reset_lane(self) -> None:
        self.current, self.current_nodes, self.current_created = [], [], []

    def _split_last_lane(self) -> None:
        """把最后一条通道在**每个点**处切开 → 每相邻两点一条通道。

        推荐做法就是多段：单段被挡只影响那一段（map_server 是按整条边判可行性的），
        而且中间点从此就是可路由的路口/岔路。
        """
        if not self.lanes:
            self.status = "no lane to split"
            return
        src = self.lanes[-1]
        names = src.get("via") or [src["start"], src["end"]]
        if len(names) < 3:
            self.status = f"lane {src['start']}->{src['end']} has 2 points, nothing to split"
            return
        # 安全性：折线里若有"不在节点表里"的形状点，切开会丢掉它们 → 拒绝
        node_pts = {(round(self.nodes[n]["x"], 3), round(self.nodes[n]["y"], 3))
                    for n in names}
        alien = [p for p in src["points"]
                 if (round(p[0], 3), round(p[1], 3)) not in node_pts]
        if alien:
            self.status = (f"lane has {len(alien)} shape point(s) that are not nodes "
                           f"-- split would drop them")
            return
        self.lanes.pop()
        made = []
        for a, b in zip(names, names[1:]):
            self.lanes.append({
                "points": [(self.nodes[a]["x"], self.nodes[a]["y"]),
                           (self.nodes[b]["x"], self.nodes[b]["y"])],
                "start": a, "end": b, "via": [a, b],
                "one_way": src["one_way"], "speed": src["speed"],
                "corridor": src["corridor"],
            })
            made.append(f"{a}->{b}")
        self.status = (f"split {src['start']}->{src['end']} into {len(made)} lanes")
        print(f"[split] {src['start']}->{src['end']} ({len(names)} points) "
              f"-> {len(made)} lanes: {', '.join(made)}")
        print("        重新保存后，map_server 会逐条判可行性：单段被挡只剔除那一段")

    def _finish_lane(self) -> None:
        if len(self.current_nodes) < 2:
            if self.current_nodes:
                self.status = "lane needs >= 2 points, discarded"
            self._reset_lane()
            return
        names = list(self.current_nodes)
        start, end = names[0], names[-1]
        if start == end:
            self.status = "lane start == end (self loop), discarded"
            self._reset_lane()
            return
        pts = [(self.nodes[n]["x"], self.nodes[n]["y"]) for n in names]
        self.lanes.append({"points": pts, "start": start, "end": end, "via": names,
                           "one_way": False, "speed": 1.0, "corridor": 0.6})
        self.status = f"lane {start} -> {end} finished ({len(names)} points)"
        self._reset_lane()

    def _finish_zone(self) -> None:
        if len(self.zone_current) < 3:
            if self.zone_current:
                self.status = "zone needs >= 3 vertices, discarded"
            self.zone_current = []
            return
        while True:
            self._zone_counter += 1
            name = f"Z{self._zone_counter}"
            if all(z["name"] != name for z in self.zones):
                break
        self.zones.append({"name": name, "type": "forbidden", "value": 0.3,
                           "points": list(self.zone_current)})
        self.status = f"zone {name} closed ({len(self.zone_current)} vertices, `t` toggles type)"
        self.zone_current = []

    # ------------------------------------------------------------- 绘制
    def redraw(self) -> None:
        for art in list(self.ax.lines) + list(self.ax.patches) + list(self.ax.texts):
            art.remove()
        bad = 0
        for lane in self.lanes:
            ok = lane_free(self.grid, lane["points"], self.half_len, self.half_wid,
                           self.zones, self.zone_inflate)
            if not ok:
                bad += 1
            xs = [p[0] for p in lane["points"]]
            ys = [p[1] for p in lane["points"]]
            color = "#d62728" if not ok else ("#ff9900" if lane["one_way"] else "#00b0f0")
            self.ax.plot(xs, ys, "-", lw=2.2, zorder=3, color=color)
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
        hint = ""
        if self.mode == "delete":
            kind, obj = self.hover
            if kind == "node" and obj in self.nodes:
                n = self.nodes[obj]                                # type: ignore[index]
                self.ax.plot([n["x"]], [n["y"]], "o", ms=17, mfc="none",
                             mec="#d62728", mew=2.2, zorder=8)
                hint = f"  |  DELETE point {obj}"
            elif kind == "lane":
                xs = [p[0] for p in obj["points"]]                # type: ignore[index]
                ys = [p[1] for p in obj["points"]]                # type: ignore[index]
                self.ax.plot(xs, ys, "-", lw=5.0, color="#d62728", alpha=0.55, zorder=8)
                hint = f"  |  DELETE lane {obj['start']}->{obj['end']}"  # type: ignore[index]
            elif kind == "zone":
                xs = [p[0] for p in obj["points"]]                # type: ignore[index]
                ys = [p[1] for p in obj["points"]]                # type: ignore[index]
                self.ax.plot(xs + [xs[0]], ys + [ys[0]], "-", lw=4.0, color="#d62728",
                             alpha=0.6, zorder=8)
                hint = f"  |  DELETE zone {obj['name']}"               # type: ignore[index]
            else:
                hint = "  |  nothing under the cursor"
        self.ax.set_title(
            f"[{self.mode}] {len(self.lanes)} lanes / {len(self.nodes)} points"
            + (f" / {len(self.zones)} zones" if self.zones else "")
            + (f"  |  WARNING: {bad} lane(s) blocked (red)" if bad else "")
            + hint
            + f"  |  {self.status}")
        self.fig.canvas.draw_idle()

    # ------------------------------------------------------------- 保存
    def _lanes_using(self, name: str) -> List[Dict]:
        return [l for l in self.lanes
                if name in [l["start"], l["end"]] or name in l.get("via", [])]

    def _components(self) -> List[List[str]]:
        """按通道连通性分组（无向）。只统计**参与通道的节点**：孤立单点（比如刚放的站点）
        不算“网络不连通”（那是另一回事，save() 里单独提示）。"""
        used = [n for n in self.nodes if self._lanes_using(n)]
        parent: Dict[str, str] = {n: n for n in used}

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
        for n in used:
            groups.setdefault(find(n), []).append(n)
        return list(groups.values())

    def save(self) -> None:
        bad = []
        for lane in self.lanes:
            if not lane_free(self.grid, lane["points"], self.half_len,
                             self.half_wid, self.zones, self.zone_inflate):
                bad.append(f"{lane['start']}->{lane['end']}")

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
        print(f"[save] {self.routes_file}: {len(self.nodes)} nodes / {len(self.lanes)} "
              f"lanes / {len(self.zones)} zones / total {total:.1f} m")
        if self.zones:
            forb = sum(1 for z in self.zones if z["type"] == "forbidden")
            spd = len(self.zones) - forb
            print(f"[zones] forbidden {forb} (map_server inflates by the vehicle "
                  f"circumscribed radius and burns them into the global map); "
                  f"speed_limit {spd} (queried by the speed planner)")

        seen: Dict[Tuple[str, str], int] = {}
        for lane in self.lanes:
            key = (lane["start"], lane["end"])
            seen[key] = seen.get(key, 0) + 1
        dup = [f"{a}->{b} x{c}" for (a, b), c in seen.items() if c > 1]
        if dup:
            print(f"[warn] duplicate lanes: {', '.join(dup)} (drawn twice?)")

        if bad:
            print(f"[warn] these lanes are blocked (red in the window): {', '.join(bad)}")
            print("       either an obstacle is in the way, or a forbidden zone covers it;")
            print("       move the lane or tighten the zone.")
        comps = self._components()
        used = {n for n in self.nodes if self._lanes_using(n)}
        standalone = [n for n in self.nodes if n not in used]
        if standalone:
            print(f"[info] {len(standalone)} standalone point(s) not connected to any "
                  f"lane: {', '.join(standalone)}")
        if len(comps) > 1:
            print(f"[warn] the network has {len(comps)} disconnected components; the "
                  f"planner can only find routes inside one component:")
            for i, comp in enumerate(comps, 1):
                print(f"       #{i}: {', '.join(comp)}")
            print("       connect them by clicking an existing point of the other part")
            print("       (points within --snap metres are reused automatically).")
        self.status = (f"saved ({len(self.lanes)} lanes)"
                       + (f", WARNING {len(bad)} blocked" if bad else "")
                       + (f", WARNING {len(comps)} components" if len(comps) > 1 else ""))


# --------------------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description="pnc_2d route network editor")
    ap.add_argument("--map-dir", default="/home/gmd/rcs/maps/go2_sim_factory",
                    help="station map directory (map.yaml + map.pgm)")
    ap.add_argument("--routes", default="",
                    help="output file (default <map-dir>/routes.yaml)")
    ap.add_argument("--lane-length", type=float, default=0.70, help="vehicle length [m]")
    ap.add_argument("--lane-width", type=float, default=0.40, help="vehicle width [m]")
    ap.add_argument("--margin", type=float, default=0.05, help="safety margin [m]")
    ap.add_argument("--snap", type=float, default=0.30,
                    help="reuse an existing point within this radius [m]")
    ap.add_argument("--zone-inflate", type=float, default=0.05,
                    help="forbidden-zone inflation [m]; <0 = auto (vehicle "
                         "circumscribed radius). Keep it equal to map_server's "
                         "zones.inflate, otherwise the editor and map_server "
                         "disagree")
    args = ap.parse_args()

    routes = args.routes or os.path.join(args.map_dir, "routes.yaml")
    editor = RouteEditor(args.map_dir, routes, args.lane_length, args.lane_width,
                         args.margin, args.snap, args.zone_inflate)
    print(f"[map] {args.map_dir} | {editor.grid.w}x{editor.grid.h} @ {editor.grid.res} m "
          f"| x[{editor.grid.x0:.2f},{editor.grid.x1:.2f}] "
          f"y[{editor.grid.y0:.2f},{editor.grid.y1:.2f}]")
    print(f"[out] {routes}")
    print(f"[zones] forbidden zones are inflated by {editor.zone_inflate:.3f} m "
          f"in the passability check (map_server zones.inflate must match)")
    editor.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
