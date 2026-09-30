# pnc_2d/scripts — MPC / NMPC 轨迹跟踪 demo

## 运行环境（重要）
- 必须用 `/home/gmd/r41_ws/.venv/bin/python3`：numpy 2.5.2 / matplotlib 3.11.1 /
  osqp 1.1.3 / casadi 3.8.1 / scipy 1.18.1
- 系统 python3.10 **不能**用：没有 osqp/casadi，且 apt matplotlib 3.5 与 pip numpy 2.2.6
  ABI 冲突（`AttributeError: _ARRAY_API not found`）→ import matplotlib 直接崩
- 运行：`cd scripts && ../.venv/bin/python3 mpc.py --save /tmp/a.png`
  参数：`--mode progress|time`、`--save PNG`、`--no-show`
- 耗时：L-MPC 754 步 ≈ 2.5 s（3.3 ms/步，可实时）；NMPC 754 步 ≈ 38 s（50 ms/步，20 步预测）
- `src/pnc_2d` 未被 git 跟踪 → 改前先备份（legacy/*_original.py）

## 文件
- `traj_utils.py` 公共：Reference(弧长参数化参考/进度索引)、project_progress、
  integrate_plant、tracking_metrics、plot_result
- `mpc.py` 误差动力学 L-MPC + OSQP；`nmpc.py` NMPC + CasADi/IPOPT
- `legacy/mpc_original.py` / `legacy/nmpc_original.py` 改动前备份

## 已修复的坑（都是"跟踪差"的真实成因）
1. 误差动力学 MPC 必须对**偏差输入** ũ=u−u_ref 优化；把 u 当绝对量代入会丢前馈
   [vr,0,wr] → e=0 时 QP 退化为 min ũ'Pũ，输出 [0,0]（原地不动）
2. 参考轨迹必须按**弧长**采样（间隔 = vr*dt）。linspace 等参数采样会让隐含速度
   Δs/dt 与 vr 无关；400 点 2 圈 r=3 → 隐含 1.89 m/s > V_MAX 1.2 → 物理上追不上
3. 参考索引必须用**最近点投影**（进度变量+窗口）；用循环计数 step 会让参考点跑掉，
   误差无界增长且永不回同步
4. NMPC 代价中航向误差必须 `atan2(sin,cos)` 包裹；参考航向 t+π/2 会涨到 >π，
   状态估计的航向是包裹的 → 误差凭空多 2π 倍数
5. 被控对象别用显式欧拉（dt=0.05、v=1、w=1/3 → 横向 RMS 1.9 cm）；用中点法（圆弧精确）
6. OSQP 目标是 min 0.5 z'Pz + q'z → P=2H、q=2h；只填 H、h 会让 R 相对 Q 隐式放大 2 倍
7. NMPC 求解失败时不要硬编码某个速度作为回退（原版 [1.0,1/3]），保持上一拍指令 + 计数

## 实测（圆 r=3 m ×2 圈、vr=1.0、V_MAX=1.2、V_MIN=0）
| 版本 | 横向 RMS | 横向 max | 航向 RMS |
|---|---|---|---|
| 旧 L-MPC | 0.945 m | 1.360 m | 0.083 rad |
| 旧 NMPC | 0.976 m | 1.310 m | 0.081 rad |
| 新 L-MPC | 0.0000 m | 0.0004 m | 0.0000 rad |
| 新 NMPC | 0.0000 m | 0.0002 m | 0.0000 rad |
旧版 99.5% 指令顶在限幅上、平均 1.19 m/s（全油门追跑掉的参考点）。
