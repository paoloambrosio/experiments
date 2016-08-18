cmake_minimum_required(VERSION 3.0)

project(Dynamic-Load)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost "1.61" COMPONENTS filesystem system REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(main main.cpp)
target_link_libraries(main ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

add_library(plugin1 MODULE plugin.cpp plugin1.cpp)
target_link_libraries(plugin1 ${Boost_SYSTEM_LIBRARY})

# link second plugin with static boost system library
unset(Boost_LIBRARIES)
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost "1.61" COMPONENTS system REQUIRED)

add_library(plugin2 MODULE plugin.cpp plugin2.cpp)
target_link_libraries(plugin2 ${Boost_SYSTEM_LIBRARY})
