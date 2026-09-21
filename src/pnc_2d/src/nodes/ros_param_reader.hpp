// ParamReader 的 ROS 实现：把 rclcpp 参数的读取适配到规划库的抽象接口。
// 规划库本身不依赖 rclcpp（见 doc/pnc2d_dev_plan.md §4.1），这一层是唯一桥梁。
//
// 【为什么这里要自己写类型转换，而不是直接 get_parameter_or<T>()】
//   1. yaml 里的字面量类型不受控：`hard_threshold: 80` 是 int，`80.0` 是 double，
//      `soft_cost_weight: 5` 是 int 而代码想读 double。rclcpp 的 Parameter 是强类型
//      的，类型不符会抛 ParameterTypeException（节点直接崩），所以这里做宽松转换：
//      数值型之间互通，读不出来才用默认值。
//   2. 未声明参数不会出现在参数表里 → 用 has_parameter() 先判存在，避免抛异常。
//
// ⚠ 配套要求（节点侧）：仅开 allow_undeclared_parameters 不够 —— get_parameter_or()
//   对未声明参数不会去查覆盖项，会静默返回默认值。节点必须用
//   automatically_declare_parameters_from_overrides(true)，让 yaml/命令行里的键
//   真正被声明。

#pragma once

#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "pnc_2d/core/param_reader.hpp"

namespace pnc_2d {

class RosParamReader : public ParamReader {
public:
  explicit RosParamReader(rclcpp::Node & node) : node_(node) {}

  bool getBool(const std::string & key, bool def) const override
  {
    const rclcpp::Parameter * p = find(key);
    if (p == nullptr) return def;
    switch (p->get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return p->as_bool();
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return p->as_int() != 0;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return p->as_double() != 0.0;
      default:
        warnType(key, "bool");
        return def;
    }
  }

  int getInt(const std::string & key, int def) const override
  {
    const rclcpp::Parameter * p = find(key);
    if (p == nullptr) return def;
    switch (p->get_type()) {
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return static_cast<int>(p->as_int());
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return static_cast<int>(std::llround(p->as_double()));
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return p->as_bool() ? 1 : 0;
      case rclcpp::ParameterType::PARAMETER_STRING:
        return std::atoi(p->as_string().c_str());
      default:
        warnType(key, "int");
        return def;
    }
  }

  double getDouble(const std::string & key, double def) const override
  {
    const rclcpp::Parameter * p = find(key);
    if (p == nullptr) return def;
    switch (p->get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return p->as_double();
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return static_cast<double>(p->as_int());
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return p->as_bool() ? 1.0 : 0.0;
      case rclcpp::ParameterType::PARAMETER_STRING:
        return std::atof(p->as_string().c_str());
      default:
        warnType(key, "double");
        return def;
    }
  }

  std::string getString(const std::string & key, const std::string & def) const override
  {
    const rclcpp::Parameter * p = find(key);
    if (p == nullptr) return def;
    if (p->get_type() == rclcpp::ParameterType::PARAMETER_STRING) return p->as_string();
    warnType(key, "string");
    return def;
  }

  std::vector<double> getDoubleArray(const std::string & key,
                                     const std::vector<double> & def) const override
  {
    const rclcpp::Parameter * p = find(key);
    if (p == nullptr) return def;
    switch (p->get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        return p->as_double_array();
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
        const auto v = p->as_integer_array();
        return std::vector<double>(v.begin(), v.end());
      }
      case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY: {
        const auto v = p->as_byte_array();
        return std::vector<double>(v.begin(), v.end());
      }
      default:
        warnType(key, "double[]");
        return def;
    }
  }

private:
  /// 取参数；未声明返回 nullptr（不抛异常）
  const rclcpp::Parameter * find(const std::string & key) const
  {
    if (!node_.has_parameter(key)) return nullptr;
    param_cache_ = node_.get_parameter(key);
    return &param_cache_;
  }

  void warnType(const std::string & key, const char * want) const
  {
    RCLCPP_WARN(node_.get_logger(),
                "[planner] 参数 %s 的类型不是 %s（实际 %s）→ 用代码默认值",
                key.c_str(), want, param_cache_.get_type_name().c_str());
  }

  rclcpp::Node & node_;
  /// 用 mutable 缓存避免返回悬垂引用（ParamReader 接口是 const）
  mutable rclcpp::Parameter param_cache_;
};

}  // namespace pnc_2d
