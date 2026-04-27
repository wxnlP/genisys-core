#include <cstring>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "proxy_module/proxy_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>>
    aimrt_module_register_array[]{
        {"ProxyModule",
         []() -> aimrt::ModuleBase* {
           return new sim_proxy::proxy_module::ProxyModule();
         }},
    };

AIMRT_PKG_MAIN(aimrt_module_register_array)
