# traj/gcopter —— vendor 的第三方算法层（只读，不要在这里写业务代码）

## 来源

- 上游：`ZJU-FAST-Lab/GCOPTER`（MINCO 轨迹优化，作者 Zhepei Wang），文件头是 **MIT License**
- 实际拷贝自：`ZJU-FAST-Lab/DDR-opt` 的 `back_end/include/gcopter/`
  （该仓库整体 **GPL-3.0**，但文件里保留着上游的 MIT 头）

## 拷了什么

| 文件 | 用途 | 备注 |
|---|---|---|
| `minco.hpp` | `MINCO_S3NU`（我们用的那个）+ `MINCO_S4NU`（没用） | ★ 命名空间是 `minco`；DDR-opt 对它做过修改（新增 `setHConditions` / `setTConditions` 等），**不要以为它就是上游原文** |
| `trajectory.hpp` | `Trajectory<Degree, Dim>`：`Trajectory<5,2>` = 5 阶多项式 × 2 维 | ★ **全局命名空间**（无 namespace）；依赖 `root_finder.hpp` |
| `root_finder.hpp` | `trajectory.hpp` 的依赖 | |
| `lbfgs.hpp` | L-BFGS（Stage A 平滑用） | ⚠ 入参是 `Eigen::VectorXd&`，**不是** C 风格的 `double*` + `n` |

没拷：`sdlp.hpp`（几何约束 QP，用不到）、`MINCO_S4NU`、ICR / 轮速相关部分。

## 许可证判定（写给将来，重要）

- 上游 gcopter 是 **MIT**；但**这一份是从 DDR-opt 拷的**，而 DDR-opt 对 `minco.hpp` 有修改，
  且 DDR-opt 仓库整体是 **GPL-3.0** ⇒ **这一份按 GPL-3.0 处理**。
- 用户 2026-09-24 明确：项目**不商用**，可以使用原版代码。
- ★ 隔离措施：第三方代码**只允许放在本目录**。将来若要闭源商用，只需把这一份
  换回上游 GCOPTER 的 MIT 版本、自己重做那几处修改（`setHConditions`/`setTConditions`）
  —— **替换本目录即可，产品代码一行不用动**。
- 我们自己的东西（`(θ,s)` 映射、Simpson 积分、时间解析缩放、终点修正、SDF 平滑）
  都在 `pnc_2d/traj/minco_optimizer.{hpp,cpp}` 与 `pnc_2d/core/trajectory_optimizer.hpp`，
  **不在本目录**。

## 改动

- **对文件零改动**（原文件里的 `#include "gcopter/..."` 保持不变），代价是 CMake 里
  多一个 include 路径 `include/pnc_2d/traj`。这样将来重新 vendor 可以直接覆盖，
  不需要重新打补丁、也不会和上游 diff 打架。

## 用法（照抄 `minco_optimizer.cpp`）

```cpp
minco::MINCO_S3NU minco_opt;                       // ⚠ 命名空间是 minco（不是 gcopter）
minco_opt.setConditions(head, tail, n_piece, Eigen::Vector2d(1.0, 1.0));  // head/tail: 2×3 PVA
minco_opt.setParameters(inPs, ts);      // inPs: 2×(n_piece−1) 内点；ts: n_piece 段时间
Trajectory<5, 2> traj;                 // ⚠ Trajectory 在**全局**命名空间（trajectory.hpp 无 namespace）
minco_opt.getTrajectory(traj);         // 之后 traj.getPos/getVel/getAcc/getJer(t)
```

⚠ 两个坑：
1. `ts` 里**出现 0 或负值**会在带状矩阵里制造除零（我们已在 `solveMinco` 里挡住）；
2. `Trajectory<5,2>` 的模板参数是 **(多项式阶数, 维度)**，不是 (维度, 阶数)。
