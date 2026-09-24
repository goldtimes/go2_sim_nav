// P6/M4：参考速度剖面的单测。
//
// 测什么（以及为什么不测别的）：
//   · **校验纪律**：长度不匹配 / 非单调 / NaN / 负速 ⇒ **整体作废**。
//     这一条是本层最容易被"半信半疑地用一半"的地方 —— 半份剖面比没有更危险：
//     它会安静地用错位的 v(s)，而日志上看不出来（R8/R9 那类问题的典型形态）。
//   · **查表**：线性插值的正确性 + 端点夹取 + 无效时不崩。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "pnc_2d/core/reference_profile.hpp"

namespace pnc_2d {
namespace {

/// 造一份合法剖面：s = [0, 1, 2]，v = [0, 3, 6]，w = [0, 0.2, 0.4]
ReferenceProfile good()
{
  ReferenceProfile p;
  std::string err;
  EXPECT_TRUE(p.set({0.0, 1.0, 2.0}, {0.0, 3.0, 6.0}, {0.0, 0.2, 0.4}, err)) << err;
  return p;
}

TEST(ReferenceProfile, RejectsMismatchedLengthsInsteadOfUsingHalfOfIt)
{
  ReferenceProfile p;
  std::string err;
  // v 少一个 ⇒ 必须**整体**作废（不能"用能对齐的那几个"）
  EXPECT_FALSE(p.set({0.0, 1.0, 2.0}, {0.0, 3.0}, {0.0, 0.2, 0.4}, err));
  EXPECT_FALSE(p.valid());
  EXPECT_FALSE(err.empty());
  std::printf("[M4] 长度不匹配：%s\n", err.c_str());
  // w 多一个
  EXPECT_FALSE(p.set({0.0, 1.0, 2.0}, {0.0, 3.0, 6.0}, {0.0, 0.2, 0.4, 0.6}, err));
  EXPECT_FALSE(p.valid());
  // 空数组（= "没有剖面"）也必须干净作废，不崩
  EXPECT_FALSE(p.set({}, {}, {}, err));
  EXPECT_FALSE(p.valid());
  // 单点
  EXPECT_FALSE(p.set({0.0}, {1.0}, {0.0}, err));
}

TEST(ReferenceProfile, RejectsNonMonotonicOrNonFiniteData)
{
  ReferenceProfile p;
  std::string err;
  // s 有相等（插值会除零）
  EXPECT_FALSE(p.set({0.0, 1.0, 1.0}, {0.0, 3.0, 6.0}, {0.0, 0.0, 0.0}, err));
  EXPECT_FALSE(p.valid());
  // s 回退
  EXPECT_FALSE(p.set({0.0, 1.0, 0.5}, {0.0, 3.0, 6.0}, {0.0, 0.0, 0.0}, err));
  // NaN / Inf
  const double nan_v = std::nan("");
  EXPECT_FALSE(p.set({0.0, 1.0, 2.0}, {0.0, nan_v, 6.0}, {0.0, 0.0, 0.0}, err));
  EXPECT_FALSE(p.set({0.0, 1.0, 2.0}, {0.0, 3.0, 6.0}, {0.0, 0.0, INFINITY}, err));
  // 负速度（不倒车 ⇒ 剖面里的 v 必须 ≥ 0）
  EXPECT_FALSE(p.set({0.0, 1.0, 2.0}, {0.0, -1.0, 6.0}, {0.0, 0.0, 0.0}, err));
  EXPECT_NE(err.find("负值"), std::string::npos);
}

TEST(ReferenceProfile, SetSucceedsThenClearInvalidates)
{
  ReferenceProfile p = good();
  ASSERT_TRUE(p.valid());
  EXPECT_EQ(p.size(), 3u);
  EXPECT_NEAR(p.length(), 2.0, 1e-12);
  p.clear();
  EXPECT_FALSE(p.valid());
  EXPECT_DOUBLE_EQ(p.speedAt(1.0), 0.0);   // 无效时不崩、返回 0
  EXPECT_DOUBLE_EQ(p.omegaAt(1.0), 0.0);
}

TEST(ReferenceProfile, InterpolatesLinearlyAndClampsAtEnds)
{
  const ReferenceProfile p = good();
  ASSERT_TRUE(p.valid());
  EXPECT_NEAR(p.speedAt(0.0), 0.0, 1e-12);
  EXPECT_NEAR(p.speedAt(1.0), 3.0, 1e-12);
  EXPECT_NEAR(p.speedAt(2.0), 6.0, 1e-12);
  // 中点插值
  EXPECT_NEAR(p.speedAt(0.5), 1.5, 1e-12);
  EXPECT_NEAR(p.speedAt(1.25), 3.75, 1e-12);
  EXPECT_NEAR(p.omegaAt(1.25), 0.25, 1e-12);
  // 越界夹取（"没有更多信息" ⇒ 用端点值，不报错、不回 0）
  EXPECT_NEAR(p.speedAt(-5.0), 0.0, 1e-12);
  EXPECT_NEAR(p.speedAt(99.0), 6.0, 1e-12);
  EXPECT_NEAR(p.omegaAt(99.0), 0.4, 1e-12);
}

TEST(ReferenceProfile, RepeatedSetReplacesPreviousContentWithoutLeftovers)
{
  // 换任务时同一对象会被反复 set ⇒ 不能残留上一份的任何数据
  ReferenceProfile p = good();
  ASSERT_TRUE(p.valid());
  std::string err;
  ASSERT_TRUE(p.set({0.0, 5.0}, {1.0, 2.0}, {0.0, 0.0}, err)) << err;
  EXPECT_EQ(p.size(), 2u);
  EXPECT_NEAR(p.length(), 5.0, 1e-12);
  EXPECT_NEAR(p.speedAt(2.5), 1.5, 1e-12);
  // put 失败之后也必须清干净（否则会继续用旧的）
  EXPECT_FALSE(p.set({0.0, 1.0}, {1.0}, {0.0}, err));
  EXPECT_FALSE(p.valid());
  EXPECT_EQ(p.size(), 0u);
}

// ---------------------------------------------------------------------------
// 访问器与 `set()` 必须是**同一份数据**
//
// 为什么单独一条：初版结构体里有 `std::vector<double> s, v, w;` 三个**公共**
// 成员（草稿遗留），而真正被 `set()` 填的是私有的 `s_/v_/w_` —— 两者同名、
// 前者**永远为空**。调用方 `*std::max_element(p.v.begin(), p.v.end())` 编译得过、
// 也测不到（旧用例只用 speedAt/omegaAt），现场却直接 SIGSEGV。
// 现在那些成员已删除（写错就编译不过），这条用例是给"将来又加一个数据副本"兜底。
// ---------------------------------------------------------------------------
TEST(ReferenceProfile, AccessorsExposeTheSameDataThatWasSet)
{
  ReferenceProfile p = good();
  ASSERT_TRUE(p.valid());
  // 长度：size() / sAxis() / speeds() / omegas() 四者必须一致
  EXPECT_EQ(p.speeds().size(), p.size());
  EXPECT_EQ(p.omegas().size(), p.size());
  EXPECT_EQ(p.sAxis().size(), p.size());
  // 内容：抽查几个点，且与插值口径一致
  ASSERT_GE(p.speeds().size(), 2u);
  EXPECT_NEAR(p.speeds().front(), p.speedAt(p.sAxis().front()), 1e-12);
  EXPECT_NEAR(p.speeds().back(), p.speedAt(p.sAxis().back()), 1e-12);
  EXPECT_NEAR(p.omegas().front(), p.omegaAt(p.sAxis().front()), 1e-12);
  // peakSpeed 必须等于 set 进去的最大 v（**空数组上不能崩**）
  double vmax = 0.0;
  for (const double v : p.speeds()) vmax = std::max(vmax, v);
  EXPECT_NEAR(p.peakSpeed(), vmax, 1e-12);
  EXPECT_GT(p.peakSpeed(), 0.0);
}

TEST(ReferenceProfile, PeakSpeedIsSafeWhenInvalid)
{
  // 无效/从未 set 过的剖面上取峰值 ⇒ 0，而不是对 end() 解引用（SIGSEGV）
  ReferenceProfile fresh;
  EXPECT_FALSE(fresh.valid());
  EXPECT_EQ(fresh.peakSpeed(), 0.0);
  EXPECT_EQ(fresh.size(), 0u);
  EXPECT_TRUE(fresh.speeds().empty());

  ReferenceProfile bad = good();
  std::string err;
  EXPECT_FALSE(bad.set({0.0, 1.0}, {1.0}, {0.0}, err));  // 长度不匹配 ⇒ 作废
  EXPECT_EQ(bad.peakSpeed(), 0.0);
  EXPECT_TRUE(bad.speeds().empty());
}

// ---------------------------------------------------------------------------
// 数值噪声容差（2026-09-24 实测）：MINCO 末点的 v 会出现 -1e-16 这种数
// ---------------------------------------------------------------------------
TEST(ReferenceProfile, ToleratesNumericalNoiseButRejectsRealNegatives)
{
  std::string err;
  // ① 数值噪声：接受，并把负数夹成 0（否则整份剖面被丢掉 —— 现场就是这个现象）
  {
    ReferenceProfile p;
    ASSERT_TRUE(p.set({0.0, 1.0, 2.0}, {0.5, 0.3, -1.0e-16}, {0.0, 0.0, 0.0},
                      err))
        << err;
    EXPECT_TRUE(p.valid());
    EXPECT_NEAR(p.speeds().back(), 0.0, 1e-18);
    EXPECT_GE(p.speedAt(99.0), 0.0);   // 夹取后不可能查到负速度
  }
  // ② 真有意义的负速度：仍然拒绝（那是上游算错了，不能静默吃掉）
  {
    ReferenceProfile p;
    EXPECT_FALSE(p.set({0.0, 1.0, 2.0}, {0.5, -0.01, 0.3}, {0.0, 0.0, 0.0},
                       err));
    EXPECT_FALSE(p.valid());
    // 错误消息里必须点名"负值"（否则现场分不清是长度问题还是数值问题）
    EXPECT_NE(err.find("负值"), std::string::npos) << err;
  }
}

}  // namespace
}  // namespace pnc_2d
