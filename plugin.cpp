#include <iostream>
#include "plugin.hpp"

some_namespace::Plugin plugin_object;

extern "C" BOOST_SYMBOL_EXPORT some_namespace::Plugin *plugin;
some_namespace::Plugin *plugin = &plugin_object;
