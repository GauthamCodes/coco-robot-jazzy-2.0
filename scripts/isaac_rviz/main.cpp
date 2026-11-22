// Copyright 2026 Gautham Anil
// SPDX-License-Identifier: Apache-2.0
// Host the existing RViz app with explicit display-before-ROS destruction.
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <QApplication>
#include <QTimer>
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/visualizer_app.hpp"
#include "rviz_common/visualization_frame.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/display_group.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"

volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }
using RealRos = rviz_common::ros_integration::RosClientAbstraction;
class DeferredShutdown : public rviz_common::ros_integration::RosClientAbstractionIface {
 public:
  explicit DeferredShutdown(std::shared_ptr<RealRos> ros) : ros_(ros) {}
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr init(
    int argc, char **argv, const std::string &name, bool anonymous) override {
    return ros_->init(argc, argv, name, anonymous);
  }
  bool ok() override { return ros_->ok(); }
  void shutdown() override { /* main shuts ROS down after the RViz frame dies */ }
 private:
  std::shared_ptr<RealRos> ros_;
};
int main(int argc, char **argv) {
  auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> qt_arguments;
  for (auto &arg : arguments) qt_arguments.push_back(arg.data());
  int qt_argc = static_cast<int>(qt_arguments.size());
  QApplication qt(qt_argc, qt_arguments.data());
  auto ros = std::make_shared<RealRos>();
  int code = 1;
  {
    rviz_common::VisualizerApp app(std::make_unique<DeferredShutdown>(ros));
    app.setApp(&qt);
    if (!app.init(argc, argv)) return 1;
    // Keep ROS alive during normal signal-triggered GUI destruction.
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    bool drained = false;
    auto drain = [&]() {
      if (drained) return;
      drained = true;
      for (auto *widget : QApplication::topLevelWidgets()) {
        auto *frame = dynamic_cast<rviz_common::VisualizationFrame *>(widget);
        if (!frame) continue;
        auto *manager = frame->getManager();
        std::cout << "RVIZ_RENDERED_FRAMES " << manager->getFrameCount() << std::endl;
        auto *group = manager->getRootDisplayGroup();
        for (int i = 0; i < group->numDisplays(); ++i) {
          auto *display = group->getDisplayAt(i);
          std::cout << "RVIZ_DISPLAY " << display->getName().toStdString()
                    << " enabled=" << display->isEnabled() << std::endl;
        }
        // ROS entities and image render resources die while their contexts live.
        manager->stopUpdate();
        manager->removeAllDisplays();
        frame->setWindowModified(false);
      }
      std::cout << "RVIZ_DISPLAYS_DRAINED ros_ok=" << ros->ok() << std::endl;
    };
    QObject::connect(&qt, &QApplication::aboutToQuit, drain);
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
      const char *file = std::getenv("COCO_RVIZ_STOP_FILE");
      if (stop_requested || (file && std::ifstream(file).good())) {
        drain();
        QApplication::closeAllWindows();
        qt.quit();
      }
    });
    timer.start(50);
    std::cout << "RVIZ_READY" << std::endl;
    code = qt.exec();
    drain();
  }
  std::cout << "RVIZ_FRAME_DESTROYED ros_ok=" << ros->ok() << std::endl;
  ros->shutdown();
  std::cout << "RVIZ_ROS_SHUTDOWN" << std::endl;
  return code;
}
