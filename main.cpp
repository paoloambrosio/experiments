#include <iostream>
#include <boost/dll.hpp>
#include "shared.hpp"

using namespace boost;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: main [plugin ...]" << std::endl;
        exit(1);
    }

    std::vector<dll::shared_library> plugin_libs;
    for (int i=1; i<argc; i++) {
        plugin_libs.push_back(dll::shared_library(argv[i]));
    }

    for (auto & lib : plugin_libs) {
        if (lib.has("plugin")) {
            PluginApi *plugin = lib.get<PluginApi *>("plugin");
            std::cout << "Hello, " << plugin->name() << "!" << std::endl;
        } else {
            std::cout << lib.location() << " is not a plugin! I can only greet plugins." << std::endl;
        }
    }
}
