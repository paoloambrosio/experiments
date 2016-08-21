cmake_minimum_required(VERSION 3.0)

project(Dynamic-Load)

set(CMAKE_CXX_STANDARD 11)



set(Boost_USE_MULTITHREADED ON)
set(Boost_DEBUG OFF)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_USE_DEBUG_RUNTIME OFF)

set(Boost_USE_STATIC_LIBS OFF)
add_definitions(-DBOOST_ALL_NO_LIB)
add_definitions(-DBOOST_ALL_DYN_LINK)

find_package(Boost "1.61" COMPONENTS filesystem system REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})



add_executable(main main.cpp)
target_link_libraries(main ${Boost_LIBRARIES})

add_library(plugin1 MODULE plugin.cpp plugin1.cpp)
target_link_libraries(plugin1 ${Boost_LIBRARIES})

add_library(plugin2 MODULE plugin.cpp plugin2.cpp)
target_link_libraries(plugin2 ${Boost_LIBRARIES})
