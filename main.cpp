#include <iostream>
#include <boost/dll.hpp>
#include "shared.hpp"

using namespace boost;

int main(int argc, char** argv) {
  if (argc <= 2) {
    std::cout << "Usage: main [plugin ...]" << std::endl;
    exit(1);
  }

  for (int i=1; i<argc; i++) {
    auto plugin = dll::import<PluginApi>(argv[i], "plugin", dll::load_mode::append_decorations);
    std::cout << "Hello, " << plugin->name() << "!" << std::endl;
  }
}
