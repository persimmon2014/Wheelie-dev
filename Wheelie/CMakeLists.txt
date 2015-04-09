cmake_minimum_required(VERSION 2.8)

# Enables testing framework.
# include(CTest)

project(Wheelie)

# add_definitions(-DHAVE_CAIRO)

# Implementation files
add_library(Wheelie SHARED
  src/hybrid-draw.cpp
  src/hybrid-micro.cpp
  src/libhybrid-common.cpp
  src/hybrid-macro.cpp
  src/hybrid-sim.cpp
  src/timer.cpp
  #src/wheelie.cpp
  )

# Adds an executable for unit tests.
# add_executable(WheelieUnitTests wheelie_unit_tests.cpp)

set_target_properties(Wheelie PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/Wheelie/lib)

# Adds install targets.
# install(/lib wheelie)
# install_files(/include/wheelie .hpp)

# Util headers
# include_directories(./RoadieUtil/)

# Link the stochastic graph library
# target_link_libraries(AwareRouting StochGraph)
# add_subdirectory(StochGraph)

# Modules available in 'cmake --help-module-list'
# However, I could not find any of the libraries in the modules.
# find_package (ImageMagick)
# if (IMAGEMAGICK_FOUND)
#   include_directories(${IMAGEMAGICK_INCLUDE_DIRS})
#   target_link_libraries (AwareRouting ${IMAGEMAGICK_LIBRARIES})
# endif (IMAGEMAGICK_FOUND)

# Another approach, which did not work for me, should be something like
# find_package(PkgConfig)
# pkg_check_modules(PC_LIBXML QUIET libxml-2.0)
# find_path(LIBXML2_INCLUDE_DIR libxml/xpath.h
#           HINTS ${PC_LIBXML_INCLUDEDIR} ${PC_LIBXML_INCLUDE_DIRS}
#           PATH_SUFFIXES libxml2 )
# find_library(LIBXML2_LIBRARY NAMES xml2 libxml2)
# find_library(LIBXMLPP_LIBRARY NAMES xml++ libxml++)

# Other library includes
include_directories(./include/)
include_directories(${CMAKE_SOURCE_DIR}/Roadie/include/)
include_directories(/usr/include/glibmm-2.4/)
include_directories(/usr/lib/x86_64-linux-gnu/glibmm-2.4/include/)
include_directories(/usr/include/glib-2.0/)
include_directories(/usr/lib/x86_64-linux-gnu/glib-2.0/include/)
include_directories(/usr/include/libxml++-2.6/)
include_directories(/usr/lib/libxml++-2.6/include/)
# include_directories(/usr/include/cairo/)
# include_directories(/usr/include/ImageMagick/)
# include_directories(/usr/include/libxml2/)

set (EXTERNAL_LIBS
  ${CMAKE_SOURCE_DIR}/Roadie/lib/libRoadie.so
  /usr/lib/x86_64-linux-gnu/libglibmm-2.4.so
  /usr/lib/libxml++-2.6.so
  /usr/lib/x86_64-linux-gnu/libboost_system.so
  /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
  /usr/lib/x86_64-linux-gnu/libGLEW.so
  /usr/lib/x86_64-linux-gnu/libGLU.so
  /usr/lib/x86_64-linux-gnu/mesa/libGL.so)

# Sets the CMAKE_THREAD_LIBS_INIT variable to link pthreads.
find_package(Threads)

# And links
target_link_libraries(Wheelie ${EXTERNAL_LIBS})
# target_link_libraries(WheelieUnitTests
#   ${EXTERNAL_LIBS}
#   wheelie
#   /usr/src/gtest/libgtest.a
# ${CMAKE_THREAD_LIBS_INIT})

# target_link_libraries(roadie /usr/lib/libproj.so)
# target_link_libraries(AwareRouting /usr/local/lib/libhybrid.so)
# target_link_libraries(AwareRouting /usr/local/lib/libroad.so)
# target_link_libraries(AwareRouting /usr/lib/x86_64-linux-gnu/libfltk.so)
# target_link_libraries(AwareRouting /usr/lib/x86_64-linux-gnu/libfltk_gl.so)
# target_link_libraries(roadie /usr/lib/x86_64-linux-gnu/libcairo.so)

# Tells cmake about the unit tests.
# add_test(TestsForWheelie WheelieUnitTests)

# Turns on the debugging information.
# TODO This should have an option to turn off.
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -ggdb")

# This turns on the C++11 options.
# TODO This should be set globally.
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag(--std=c++11 SUPPORTS_STD_CXX11)
    check_cxx_compiler_flag(--std=c++0x SUPPORTS_STD_CXX01)
    if(SUPPORTS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --std=c++11")
    elseif(SUPPORTS_STD_CXX01)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++0x")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --std=c++0x")
    else()
        message(ERROR "Compiler does not support --std=c++11 or --std=c++0x.")
    endif()
endif()
