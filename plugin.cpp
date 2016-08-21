#include <iostream>
#include "plugin.hpp"

some_namespace::Plugin plugin_object;
extern "C" BOOST_SYMBOL_EXPORT some_namespace::Plugin *plugin = &plugin_object;

some_namespace::Plugin::Plugin() {
  std::cout << "Loading " << name() << std::endl;
}

some_namespace::Plugin::~Plugin() {
  std::cout << "Unloading " << name() << std::endl;
}
