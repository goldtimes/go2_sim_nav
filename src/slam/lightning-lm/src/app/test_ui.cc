//
// Created by xiang on 25-8-27.
//

#include "common/log.h"
#include "ui/pangolin_window.h"

#include <gflags/gflags.h>

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler.h>

int main(int argc, char** argv) {
    lightning::logging::Init();

    google::ParseCommandLineFlags(&argc, &argv, true);

    lightning::ui::PangolinWindow ui;
    ui.Init();

    while (!ui.ShouldQuit()) {
        sleep(1);
    }

    ui.Quit();
    lightning::logging::Shutdown();

    return 0;
}
