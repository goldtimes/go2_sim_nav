// A1 专用 Gazebo Classic 关节 PD 插件（带力矩钳位）。
//
// 复刻上游 ROS 1 unitree_legged_control/UnitreeJointController 的控制律：
//     tau_i = clamp(kp_i * (q_des_i - q_i) - kd_i * dq_i, -tau_max, +tau_max)
//
// 不用 gazebo_ros2_control 的原因见 package.xml：它的位置环 PID 不钳位，
// 会把远超关节上限的力矩灌进 ODE，把机器人甩飞。

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace a1_gazebo_plugins
{

class JointPDPlugin : public gazebo::ModelPlugin
{
public:
  JointPDPlugin() = default;
  ~JointPDPlugin() override = default;

  // Gazebo 会把 Load 抛出的异常吞掉，只打印
  // "Exception occured in the Load function ... This plugin will not run"，
  // 完全不说是哪里出错。这里自己接住并打印原因，省得反复盲猜。
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    std::cerr << "[a1_joint_pd] Load 被调用" << std::endl;
    try {
      LoadImpl(model, sdf);
    } catch (const std::exception & e) {
      std::cerr << "[a1_joint_pd] Load 失败: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "[a1_joint_pd] Load 失败: 未知异常" << std::endl;
    }
  }

  void LoadImpl(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    model_ = model;

    std::string ns = "/";
    if (sdf->HasElement("ros")) {
      auto ros = sdf->GetElement("ros");
      if (ros->HasElement("namespace")) {
        ns = ros->Get<std::string>("namespace");
      }
    }
    std::string cmd_topic = "joint_group_controller/commands";
    if (sdf->HasElement("command_topic")) {
      cmd_topic = sdf->Get<std::string>("command_topic");
    }
    std::string js_topic = "joint_states";
    if (sdf->HasElement("joint_state_topic")) {
      js_topic = sdf->Get<std::string>("joint_state_topic");
    }
    tau_max_ = sdf->HasElement("tau_max") ? sdf->Get<double>("tau_max") : 33.5;
    control_rate_ = sdf->HasElement("update_rate") ? sdf->Get<double>("update_rate") : 500.0;

    // 关节表：<joints><joint name="..." kp="..." kd="..."/>...</joints>
    if (!sdf->HasElement("joints")) {
      gzerr << "[a1_joint_pd] 缺少 <joints> 配置，插件不生效" << std::endl;
      return;
    }
    auto joints_elem = sdf->GetElement("joints");
    for (auto joint_elem = joints_elem->GetElement("joint"); joint_elem;
         joint_elem = joint_elem->GetNextElement("joint")) {
      // ⚠️ name/kp/kd 是 XML 属性，必须用 GetAttribute；
      //    用 Get<T>("kp") 会当成子元素去查，查不到就抛异常，
      //    而 Gazebo 只会打印 "Exception occured in the Load function" 不说明原因。
      const std::string name = joint_elem->GetAttribute("name")->GetAsString();
      auto joint = model_->GetJoint(name);
      if (!joint) {
        gzerr << "[a1_joint_pd] 模型里找不到关节: " << name << std::endl;
        continue;
      }
      joint_names_.push_back(name);
      joints_.push_back(joint);
      // sdformat9 的 Param 只有 Get(T&) 这种出参写法，没有 Get<T>()
      double kp = 0.0;
      double kd = 0.0;
      joint_elem->GetAttribute("kp")->Get(kp);
      joint_elem->GetAttribute("kd")->Get(kd);
      kp_.push_back(kp);
      kd_.push_back(kd);
      q_des_.push_back(0.0);
    }
    if (joints_.empty()) {
      gzerr << "[a1_joint_pd] 没有可用关节，插件不生效" << std::endl;
      return;
    }

    // 必须用 gazebo_ros::Node::Get(sdf)：它负责 rclcpp::init、命名空间与 remapping，
    // 并把节点交给 gazebo_ros 自己的 executor（所以下面 OnUpdate 里**不要**再 spin，
    // 否则会抛 "Node has already been added to an executor"）。
    // 自己 new rclcpp::Node 会让 rcl 上下文/生命周期与 gazebo_ros 冲突，
    // 实测表现为 gzserver 在 Load 之后直接退出。
    node_ = gazebo_ros::Node::Get(sdf);

    cmd_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      cmd_topic, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != joints_.size()) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "目标关节数 %zu 与配置的 %zu 不一致，忽略", msg->data.size(), joints_.size());
          return;
        }
        for (size_t i = 0; i < joints_.size(); ++i) {
          q_des_[i] = msg->data[i];
        }
        received_command_ = true;
      });

    js_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(js_topic, 10);

    last_update_time_ = model_->GetWorld()->SimTime();
    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&JointPDPlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "[a1_joint_pd] 已加载: " << joints_.size() << " 个关节, tau_max="
          << tau_max_ << " N·m, namespace=" << ns
          << ", topic=" << cmd_topic << std::endl;
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo & /*info*/)
  {
    const auto now = model_->GetWorld()->SimTime();
    const double dt = (now - last_update_time_).Double();
    const double period = (control_rate_ > 0.0) ? 1.0 / control_rate_ : 0.0;
    if (dt >= period) {
      last_update_time_ = now;

      auto js = std::make_unique<sensor_msgs::msg::JointState>();
      js->header.stamp = rclcpp::Time(now.sec, now.nsec);
      js->name = joint_names_;

      for (size_t i = 0; i < joints_.size(); ++i) {
        const double q = joints_[i]->Position(0);
        const double dq = joints_[i]->GetVelocity(0);

        double tau = kp_[i] * (q_des_[i] - q) - kd_[i] * dq;
        tau = std::max(-tau_max_, std::min(tau_max_, tau));   // ← 关键：力矩钳位
        joints_[i]->SetForce(0, tau);

        js->position.push_back(q);
        js->velocity.push_back(dq);
        js->effort.push_back(tau);
      }
      js_pub_->publish(std::move(js));
    }
  }

  gazebo::physics::ModelPtr model_;
  std::vector<gazebo::physics::JointPtr> joints_;
  std::vector<std::string> joint_names_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<double> q_des_;
  double tau_max_{33.5};
  double control_rate_{500.0};
  bool received_command_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;

  gazebo::event::ConnectionPtr update_conn_;
  gazebo::common::Time last_update_time_;
};

}  // namespace a1_gazebo_plugins

GZ_REGISTER_MODEL_PLUGIN(a1_gazebo_plugins::JointPDPlugin)
