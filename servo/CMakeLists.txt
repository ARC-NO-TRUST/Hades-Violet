cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(csse4011Project)

# Collect all the source files in app/prac3/src
FILE(GLOB app_sources src/*.c)

# Collect all the source files in lib. Note that
# the path given needs to be relative to the
# CMakeLists root, which is app/project here,
# hence the ../../lib.c.
target_sources(app PRIVATE src/main.c)

# Tell CMake where our header files are
target_include_directories(app PRIVATE ../include)