// 参数读取抽象：让规划库与 ROS 解耦。
//
// 为什么要有这一层：规划库如果直接调 rclcpp::Node::get_parameter，
//   ① 单元测试必须起一个 ROS 节点（慢、还要 roscore/rclcpp::init）；
//   ② 将来把规划器放进 MPC 节点内进程调用时，参数来源会变成另一套。
// 所以库只认 ParamReader，节点里用 RosParamReader 适配，单测用 MemoryParamReader。

#pragma once

#include <map>
#include <string>
#include <vector>

namespace pnc_2d {

class ParamReader {
public:
  virtual ~ParamReader() = default;

  /// 所有 get* 都是"取不到就用默认值"，与 ROS 2 的 get_parameter_or 语义一致。
  virtual bool getBool(const std::string & key, bool def) const = 0;
  virtual int getInt(const std::string & key, int def) const = 0;
  virtual double getDouble(const std::string & key, double def) const = 0;
  virtual std::string getString(const std::string & key, const std::string & def) const = 0;
  virtual std::vector<double> getDoubleArray(
    const std::string & key, const std::vector<double> & def) const = 0;
};

/// 内存实现：单测与"不配置就用代码默认值"的场景。
class MemoryParamReader : public ParamReader {
public:
  void setBool(const std::string & key, bool v) { bools_[key] = v; }
  void setInt(const std::string & key, int v) { ints_[key] = v; }
  void setDouble(const std::string & key, double v) { doubles_[key] = v; }
  void setString(const std::string & key, const std::string & v) { strings_[key] = v; }
  void setDoubleArray(const std::string & key, const std::vector<double> & v) { arrays_[key] = v; }

  bool getBool(const std::string & key, bool def) const override
  {
    auto it = bools_.find(key);
    return it == bools_.end() ? def : it->second;
  }
  int getInt(const std::string & key, int def) const override
  {
    auto it = ints_.find(key);
    return it == ints_.end() ? def : it->second;
  }
  double getDouble(const std::string & key, double def) const override
  {
    auto it = doubles_.find(key);
    return it == doubles_.end() ? def : it->second;
  }
  std::string getString(const std::string & key, const std::string & def) const override
  {
    auto it = strings_.find(key);
    return it == strings_.end() ? def : it->second;
  }
  std::vector<double> getDoubleArray(
    const std::string & key, const std::vector<double> & def) const override
  {
    auto it = arrays_.find(key);
    return it == arrays_.end() ? def : it->second;
  }

private:
  std::map<std::string, bool> bools_;
  std::map<std::string, int> ints_;
  std::map<std::string, double> doubles_;
  std::map<std::string, std::string> strings_;
  std::map<std::string, std::vector<double>> arrays_;
};

}  // namespace pnc_2d
