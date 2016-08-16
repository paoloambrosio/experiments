#ifndef PLUGIN_HPP
#define PLUGIN_HPP

#include <boost/dll.hpp>
#include "shared.hpp"

namespace some_namespace {

  class Plugin : public PluginApi {
  public:
    Plugin();
    std::string name() const;
    ~Plugin();
  };
}

#endif // PLUGIN_HPP
