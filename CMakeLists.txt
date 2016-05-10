cmake_minimum_required (VERSION 2.8)
project (rapp_hazard)

find_package(Rapp REQUIRED)
find_package(RappRobots REQUIRED)

include_directories("/usr/include/eigen3")

#message("RAPP: ${Rapp_DIR}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${Rapp_DIR})
include(RappMacros)

setup_rapp()

create_rapp(
	NAME rapp_hazard
	LIBS rapp_dynamic_navigation tinyxml2
	SRCS main.cpp
)
