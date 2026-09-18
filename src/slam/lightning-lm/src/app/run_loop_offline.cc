//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>

#include "common/log.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "ui/pangolin_window.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    lightning::logging::InitFromYaml(FLAGS_config);
    if (FLAGS_input_bag.empty()) {
        LLOG_ERROR(lightning::logging::kApp, "未指定输入数据");
        lightning::logging::Shutdown();
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);

    LaserMapping lio;
    if (!lio.Init(FLAGS_config)) {
        LLOG_ERROR(lightning::logging::kApp, "failed to init lio");
        lightning::logging::Shutdown();
        return -1;
    };

    auto ui = std::make_shared<ui::PangolinWindow>();
    ui->Init();
    lio.SetUI(ui);

    auto loop = std::make_shared<LoopClosing>();
    loop->Init(FLAGS_config);

    Keyframe::Ptr cur_kf = nullptr;

    rosbag
        .AddImuHandle("imu_raw",
                      [&lio](IMUPtr imu) {
                          lio.ProcessIMU(imu);
                          return true;
                      })
        .AddPointCloud2Handle("points_raw",
                              [&lio, &cur_kf, &loop](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                                  lio.ProcessPointCloud2(cloud);
                                  lio.Run();

                                  auto kf = lio.GetKeyframe();
                                  if (cur_kf != kf) {
                                      cur_kf = kf;
                                      loop->AddKF(kf);
                                  }

                                  return true;
                              })
        .Go();

    lio.SaveMap();
    Timer::PrintAll();

    ui->Quit();

    LLOG_INFO(lightning::logging::kApp, "done");
    lightning::logging::Shutdown();

    return 0;
}