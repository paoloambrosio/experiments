#ifndef PLUGIN_HPP
#define PLUGIN_HPP

#include <boost/dll.hpp>
#include "shared.hpp"

namespace some_namespace {

  class Plugin : public PluginApi {
  public:
    const char *name() const;
  };
}

#endif // PLUGIN_HPP
