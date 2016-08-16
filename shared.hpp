#ifndef PLUGIN_API_HPP
#define PLUGIN_API_HPP

#include <string>

class PluginApi {
public:
   virtual std::string name() const = 0;

   virtual ~PluginApi() {}
};

#endif // PLUGIN_API_HPP
