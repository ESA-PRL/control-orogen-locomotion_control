PROJECT(locomotion_control)
cmake_minimum_required(VERSION 2.6)

find_package(Rock)
if (ROCK_USE_CXX11)
    rock_activate_cxx11()
endif()

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(locomotion_controlBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

