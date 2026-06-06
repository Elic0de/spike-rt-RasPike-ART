#include "app.h"

#include "app/FirmwareApplication.h"
#include "kernel_cfg.h"

namespace {

etrobo::app::FirmwareApplication* g_app = nullptr;

}  // namespace

extern "C" void main_task(intptr_t)
{
  static etrobo::app::FirmwareApplication app;
  g_app = &app;
  g_app->initialize();
  sta_cyc(CONTROL_TASK_CYC);
  ext_tsk();
}

extern "C" void control_task(intptr_t)
{
  if (g_app != nullptr) {
    g_app->runOneCycle();
  }
  ext_tsk();
}
