//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>

#include "common/log.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "ui/pangolin_window.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include <opencv2/opencv.hpp>

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_bool(show_grid_map, false, "是否展示栅格地图");

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

    g2p5::G2P5::Options map_opt;
    map_opt.online_mode_ = false;

    g2p5::G2P5 map(map_opt);
    map.Init(FLAGS_config);

    auto ui = std::make_shared<ui::PangolinWindow>();
    ui->Init();
    lio.SetUI(ui);

    Keyframe::Ptr cur_kf = nullptr;

    rosbag
        .AddImuHandle("imu_raw",
                      [&lio](IMUPtr imu) {
                          lio.ProcessIMU(imu);
                          return true;
                      })
        .AddPointCloud2Handle("points_raw",
                              [&](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                                  lio.ProcessPointCloud2(cloud);
                                  lio.Run();

                                  auto kf = lio.GetKeyframe();
                                  if (cur_kf != kf) {
                                      cur_kf = kf;

                                      // pcl::io::savePCDFile("./data/" + std::to_string(cur_kf->GetID()) + ".pcd",
                                      //                      *cur_kf->GetCloud());

                                      map.PushKeyframe(cur_kf);

                                      if (FLAGS_show_grid_map) {
                                          cv::Mat image = map.GetNewestMap()->ToCV();
                                          cv::imshow("map", image);
                                          cv::waitKey(10);
                                      }
                                  }

                                  return true;
                              })
        .Go();

    lio.SaveMap();
    cv::Mat image = map.GetNewestMap()->ToCV();
    cv::imwrite("./data/map.png", image);

    Timer::PrintAll();

    ui->Quit();

    LLOG_INFO(lightning::logging::kApp, "done");
    lightning::logging::Shutdown();

    return 0;
}