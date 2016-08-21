#ifndef PLUGIN_API_HPP
#define PLUGIN_API_HPP

class PluginApi {
public:
   virtual const char *name() const = 0;
};

#endif // PLUGIN_API_HPP
