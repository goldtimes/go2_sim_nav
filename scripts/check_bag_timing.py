#!/usr/bin/env python3
"""检查 rosbag2 里各话题的时间戳是否均匀 —— 不用播放，直接读 db3。

用法:
    python3 check_bag_timing.py <bag目录> [间隔告警阈值ms，默认150]

为什么需要它
------------
`ros2 topic hz` 量的是**消息到达间隔**，它**分不清**这两件事：
    (a) 运行时卡了（传输/调度/磁盘）
    (b) 录制的时候数据本身就录成这样
两者在 `ros2 topic hz` 的 `min/max` 上长得一模一样。

实际踩过的坑：
    某个包 `ros2 topic hz` 报 `min 0.001s  max 0.797s`，
    查数据发现包里最长的时间戳间隔**就是 797.7ms** —— 完全是录制时的问题，
    跟运行环境、DDS、socket 缓冲都没关系。
    同一时期另一个包报 `max 0.127s`，它的数据里最长间隔是 126.6ms，也正好对上。

判断「运行时是否卡顿」必须用**到达间隔 − 时间戳间隔**：
    arrival_dt - stamp_dt = backlog
数据本身的不均匀会在这个差值里被抵消掉。
lightning 的 `[lidar] ... dt ...` 列和 `MonitorRecvHealth` 做的就是这个。

用法示例
--------
    python3 scripts/check_bag_timing.py ~/lio_data_stair
    python3 scripts/check_bag_timing.py ~/r41_ws/lio_data0909 150
"""

import glob
import os
import sqlite3
import statistics
import sys


def check(bag_dir, threshold_ms):
    bag_dir = bag_dir.rstrip('/')
    db3s = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))
    if not db3s:
        print(f'{bag_dir}: 找不到 .db3 文件')
        return 1

    total_mb = sum(os.path.getsize(f) for f in db3s) / 1e6
    print(f'=== {bag_dir}  ({total_mb:.0f} MB, {len(db3s)} 个 db3)')

    con = sqlite3.connect(f'file:{db3s[0]}?mode=ro', uri=True)
    for topic_id, name in con.execute('SELECT id, name FROM topics ORDER BY id'):
        ts = [r[0] for r in
              con.execute('SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp', (topic_id,))]
        if len(ts) < 2:
            print(f'  {name}: 只有 {len(ts)} 条，跳过')
            continue

        gaps = [(ts[i] - ts[i - 1]) / 1e6 for i in range(1, len(ts))]
        med = statistics.median(gaps)
        span = (ts[-1] - ts[0]) / 1e9
        big = [(ts[i] / 1e9, gaps[i - 1]) for i in range(1, len(ts)) if gaps[i - 1] > threshold_ms]

        print(f'  {name}')
        print(f'    n={len(ts)}  span={span:.1f}s  中位间隔={med:.1f}ms  实际频率={1000 / med:.1f}Hz')
        print(f'    间隔 ms:  min {min(gaps):.1f}  /  med {med:.1f}  /  max {max(gaps):.1f}')
        print(f'    超过 {threshold_ms:.0f}ms 的处数: {len(big)}')
        print(f'    -> {"OK（均匀）" if not big else "!! 数据本身就不均匀，别拿它评估运行时性能"}')

        if big:
            t0 = ts[0] / 1e9
            for t, g in big[:10]:
                print(f'       t+{t - t0:8.3f}s   gap {g:8.1f} ms')
            if len(big) > 10:
                print(f'       ... 还有 {len(big) - 10} 处')
        print()

    return 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    sys.exit(check(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 150.0))
