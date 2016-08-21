Experiment on how to implement plugins in C++

DONE
 - Linked Boost system static in one library and dynamic in the other


TODO
 - Try with different runtimes all at the same time
   - different compilers and runtimes
     - eventually got mingw to load msvc libs but only sharing char *
     - msvc debug does not work even with its dlls, release works but not with mingw dlls
   - different architectures (x32/x64)
   - different variant (debug/release)
   - links
     - http://siomsystems.com/mixing-visual-studio-versions/
     - http://www.drdobbs.com/cpp/building-your-own-plugin-framework-part/204202899
     - http://www.codeproject.com/Articles/20648/DynObj-C-Cross-Platform-Plugin-Objects
