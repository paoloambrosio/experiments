#include <iostream>
#include <boost/dll.hpp>
#include "shared.hpp"

using namespace boost;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "Usage: main <plugin>" << std::endl;
    exit(1);
  }

  auto plugin = dll::import<PluginApi>(argv[1], "plugin", dll::load_mode::append_decorations);

  std::cout << "Hello, " << plugin->name() << "!" << std::endl;
}
