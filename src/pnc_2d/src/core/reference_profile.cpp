// P6/M4：参考速度剖面的实现（口径与理由见头文件）。

#include "pnc_2d/core/reference_profile.hpp"

#include <algorithm>
#include <cmath>

namespace pnc_2d {
namespace {

bool allFinite(const std::vector<double> & a)
{
  for (const double x : a) {
    if (!std::isfinite(x)) return false;
  }
  return true;
}

}  // namespace

bool ReferenceProfile::set(const std::vector<double> & s_in,
                           const std::vector<double> & v_in,
                           const std::vector<double> & w_in, std::string & err)
{
  clear();
  const std::size_t n = s_in.size();
  if (n < 2) {
    err = "剖面点数 < 2（收到 " + std::to_string(n) + "）";
    return false;
  }
  // ★ 长度必须**完全一致**：半份剖面比没有更危险 —— 它会安静地用错位的 v(s)。
  if (v_in.size() != n || w_in.size() != n) {
    err = "剖面数组长度不一致：s=" + std::to_string(s_in.size()) +
          " v=" + std::to_string(v_in.size()) + " w=" + std::to_string(w_in.size());
    return false;
  }
  if (!allFinite(s_in) || !allFinite(v_in) || !allFinite(w_in)) {
    err = "剖面含非有限值（NaN/Inf）";
    return false;
  }
  // v ≥ 0：**容差 + 夹取**（2026-09-24 实测）。
  //   MINCO 的末点 v 会出现 -1e-16 这种**数值噪声**（(θ,s) 末端 s̈ 的求解残差）。
  //   为它把整份剖面作废是过严的：现象是"剖面明明算出来了却被丢掉"，而日志
  //   （`剖面速度出现负值 @i=66`）看起来像**上游给错了数据**，方向带偏。
  //   真有意义的负速度（超容差）仍然拒绝：那才是上游算错了。
  constexpr double kNegTol = 1.0e-6;
  for (std::size_t i = 0; i < n; ++i) {
    if (v_in[i] < -kNegTol) {
      err = "剖面速度负值超出容差（" + std::to_string(v_in[i]) + " @i=" +
            std::to_string(i) + "，容差 " + std::to_string(kNegTol) + "）";
      return false;
    }
  }
  // s 必须**严格递增**（查表要二分/插值；相等会在插值时除零）
  for (std::size_t i = 1; i < n; ++i) {
    if (!(s_in[i] > s_in[i - 1])) {
      err = "剖面弧长非严格递增 @i=" + std::to_string(i) + "（" +
            std::to_string(s_in[i - 1]) + " → " + std::to_string(s_in[i]) + "）";
      return false;
    }
  }
  s_ = s_in;
  v_ = v_in;
  w_ = w_in;
  for (double & x : v_)   // 把容差内的负数夹成 0（后续插值/限速都用 v_）
    x = std::max(0.0, x);
  valid_ = true;
  err.clear();
  return true;
}

void ReferenceProfile::clear()
{
  s_.clear();
  v_.clear();
  w_.clear();
  valid_ = false;
}

namespace {

/// 在严格递增的 `s` 上找 `sq` 所在区间，线性插值。
/// 越界夹取到端点（不报错：越界只说明"没有更多信息"，调用方自己决定）
double interpAt(const std::vector<double> & s, const std::vector<double> & y,
                double sq)
{
  const std::size_t n = s.size();
  if (n == 0) return 0.0;
  if (sq <= s.front()) return y.front();
  if (sq >= s.back()) return y.back();
  // 二分找右端下标（s 严格递增已由 set() 保证）
  std::size_t lo = 0;
  std::size_t hi = n - 1;
  while (hi - lo > 1) {
    const std::size_t mid = (lo + hi) / 2;
    if (s[mid] <= sq) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  const double ds = s[hi] - s[lo];
  if (!(ds > 1e-12)) return y[lo];   // 理论上不会（严格递增），兜一层
  const double t = (sq - s[lo]) / ds;
  return y[lo] + (y[hi] - y[lo]) * t;
}

}  // namespace

double ReferenceProfile::speedAt(double sq) const
{
  if (!valid_) return 0.0;
  return interpAt(s_, v_, sq);
}

double ReferenceProfile::omegaAt(double sq) const
{
  if (!valid_) return 0.0;
  return interpAt(s_, w_, sq);
}

double ReferenceProfile::peakSpeed() const
{
  if (v_.empty()) return 0.0;   // 无效时**返回 0 而不是崩**
  return *std::max_element(v_.begin(), v_.end());
}

}  // namespace pnc_2d
