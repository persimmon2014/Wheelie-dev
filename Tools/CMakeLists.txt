cmake_minimum_required(VERSION 2.8)

project(Tools)

# Turns on cairo option in the code.
#add_definitions(-DHAVE_CAIRO)

# find_package(FLTK REQUIRED)
# 
# if (NOT FLTK_FOUND)
# message(FATAL_ERROR "Package FLTK not found")
# endif(NOT FLTK_FOUND)
# include_directories(${FLTK_INCLUDE_DIR})


# Adds an executable for unit tests.
add_executable(hybrid-viewer hybrid-viewer.cpp)
add_executable(hybrid hybrid.cpp)

# Other library includes
include_directories(/usr/local/include/Roadie/)
include_directories(${CMAKE_SOURCE_DIR}/Wheelie/include/)
include_directories(/usr/include/glibmm-2.4/)
include_directories(/usr/lib/x86_64-linux-gnu/glibmm-2.4/include/)
include_directories(/usr/include/glib-2.0/)
include_directories(/usr/lib/x86_64-linux-gnu/glib-2.0/include/)
include_directories(/usr/include/libxml++-2.6/)
include_directories(/usr/lib/libxml++-2.6/include/)
# include_directories(/usr/include/cairo/)
# include_directories(/usr/include/ImageMagick/)
# include_directories(/usr/include/libxml2/)

# And links
set (EXTERNAL_LIBS
  /usr/local/lib/libRoadie.so
  ${CMAKE_SOURCE_DIR}/Wheelie/lib/libWheelie.so
  /usr/lib/x86_64-linux-gnu/libglibmm-2.4.so
  /usr/lib/libxml++-2.6.so
  #/usr/lib/x86_64-linux-gnu/libcairo.so
  /usr/lib/x86_64-linux-gnu/libboost_system.so
  /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
  /usr/lib/x86_64-linux-gnu/libGLEW.so
  /usr/lib/x86_64-linux-gnu/libGLU.so
  /usr/lib/x86_64-linux-gnu/libfltk.so
  /usr/lib/x86_64-linux-gnu/libfltk_gl.so
  /usr/lib/x86_64-linux-gnu/mesa/libGL.so
  #/usr/lib/nvidia-331/libGL.so
  )


# Sets the CMAKE_THREAD_LIBS_INIT variable to link pthreads.
#find_package(Threads)

target_link_libraries(hybrid-viewer ${EXTERNAL_LIBS})
target_link_libraries(hybrid ${EXTERNAL_LIBS})

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
