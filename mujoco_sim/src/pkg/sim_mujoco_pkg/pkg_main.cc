#include <cstring>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "ctrl_module/ctrl_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>>
    aimrt_module_register_array[]{
        {"CtrlModule",
         []() -> aimrt::ModuleBase* {
           return new mujoco_sim::ctrl_module::CtrlModule();
         }},
    };

AIMRT_PKG_MAIN(aimrt_module_register_array)
