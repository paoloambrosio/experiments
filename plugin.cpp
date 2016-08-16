#include <iostream>
#include "plugin.hpp"

extern "C" BOOST_SYMBOL_EXPORT some_namespace::Plugin plugin;
some_namespace::Plugin plugin;

some_namespace::Plugin::Plugin() {
  std::cout << "Loading " << name() << std::endl;
}

some_namespace::Plugin::~Plugin() {
  std::cout << "Unloading " << name() << std::endl;
}
