#include <iostream>
#include <boost/dll.hpp>
#include "shared.hpp"

using namespace boost;

namespace some_namespace {

  class Plugin1 : public PluginApi {
  public:
      Plugin1() {
          std::cout << "Constructing Plugin1" << std::endl;
      }

      std::string name() const {
          return "World";
      }

      ~Plugin1() {
          std::cout << "Destructing Plugin1" << std::endl;
      }
  };
}

extern "C" BOOST_SYMBOL_EXPORT some_namespace::Plugin1 plugin;
some_namespace::Plugin1 plugin;
