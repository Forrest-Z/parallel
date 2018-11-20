cmake_minimum_required(VERSION 2.8.3)
project(localization)

#####################
## Compile Setting ##
#####################

add_compile_options(-std=c++17)
add_compile_options(-fpermissive)

if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -Wall -Wno-unused-variable -pthread")
else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread -fopenmp")
endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )
message("Build Type:" ${CMAKE_BUILD_TYPE} ${CMAKE_CXX_FLAGS})

##########################
## Dependencies Setting ##
##########################

set(${PROJECT_NAME}_DEPS
  nox
        tf
)

set(${PROJECT_NAME}_SYSTEM_DEPS
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPS}
  ${${PROJECT_NAME}_SYSTEM_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################
## Declare ROS messages, services and actions ##
################################################

#add_message_files(
#        FILES
#
#)

#add_service_files(
#        FILES
#
#)

#generate_messages(
#        DEPENDENCIES
#        std_msgs
#)

###################################
## catkin specific configuration ##
###################################

set(${PROJECT_NAME}_PLUGINS
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${${PROJECT_NAME}_PLUGINS}
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPS}
  DEPENDS ${${PROJECT_NAME}_SYSTEM_DEPS}
)

######################
## Build Executable ##
######################
#pragma nox_begin("build executable")
add_executable(localization_Simulator
    include/Simulator/.SimulatorModule.h src/Simulator/.SimulatorModule.cpp
    include/Simulator/Simulator.h src/Simulator/Simulator.cpp
        include/Simulator/tool/CoordinateConverter.h
        include/Simulator/tool/convert_coordinates.hpp
        src/Simulator/tool/CoordinateConverter.cpp
        src/Common/Common.cpp include/Common/Common.h)
target_link_libraries(localization_Simulator
    ${catkin_LIBRARIES}
)
set_target_properties(localization_Simulator PROPERTIES OUTPUT_NAME Simulator PREFIX "")
set_target_properties(localization_Simulator PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../.bin/localization/)

add_executable(localization_GPS
    include/GPS/.GPSModule.h src/GPS/.GPSModule.cpp
    include/GPS/GPS.h src/GPS/GPS.cpp
        include/GPS/tool/CoordinateConverter.h src/GPS/tool/CoordinateConverter.cpp
        include/GPS/type/NCOM.h src/GPS/type/NCOM.cpp
        include/GPS/type/GPSIMU.h src/GPS/type/GPSIMU.cpp
        src/Common/Common.cpp include/Common/Common.h)
target_link_libraries(localization_GPS
    ${catkin_LIBRARIES}
)
set_target_properties(localization_GPS PROPERTIES OUTPUT_NAME GPS PREFIX "")
set_target_properties(localization_GPS PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../.bin/localization/)

add_executable(localization_Repeater
    include/Repeater/.RepeaterModule.h src/Repeater/.RepeaterModule.cpp
    include/Repeater/Repeater.h src/Repeater/Repeater.cpp
        src/Common/Common.cpp include/Common/Common.h)
target_link_libraries(localization_Repeater
    ${catkin_LIBRARIES}
)
set_target_properties(localization_Repeater PROPERTIES OUTPUT_NAME Repeater PREFIX "")
set_target_properties(localization_Repeater PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../.bin/localization/)


#pragma nox_end()
#################################
## Build Plugin Inplementation ##
#################################
#pragma nox_begin("build plugin")


#pragma nox_end()
