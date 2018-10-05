cmake_minimum_required(VERSION 2.8.3)
project(autotune)

#####################
## Compile Setting ##
#####################

add_compile_options(-std=c++17)
add_compile_options(-fpermissive)

if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fopenmp")
endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )
message("Build Type:" ${CMAKE_BUILD_TYPE} ${CMAKE_CXX_FLAGS})

##########################
## Dependencies Setting ##
##########################

set(${PROJECT_NAME}_DEPS
  nox
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


add_executable(Planner
    include/Planner/.PlannerModule.h src/Planner/.PlannerModule.cpp
    include/Planner/Planner.h src/Planner/Planner.cpp
        include/Planner/type/ReferenceLine.h include/Planner/tool/TrajectoryStitcher.h src/Planner/tool/TrajectoryStitcher.cpp include/Planner/tool/DecisionMaker.h src/Planner/tool/DecisionMaker.cpp include/Planner/tool/TrafficDecider.h src/Planner/tool/TrafficDecider.cpp include/Planner/rule/Junction.h src/Planner/rule/Junction.cpp include/Planner/rule/SignalLight.h src/Planner/rule/SignalLight.cpp include/Planner/rule/Overtake.h src/Planner/rule/Overtake.cpp src/Planner/type/ReferenceLine.cpp include/Planner/impl/PlannerBase.h src/Planner/impl/PlannerBase.cpp include/Planner/impl/LatticePlanner.h src/Planner/impl/LatticePlanner.cpp include/Planner/type/SLTBoundary.h include/Planner/type/STGraph.h src/Planner/type/STGraph.cpp include/Planner/tool/lattice/LatticeGenerator.h src/Planner/tool/lattice/LatticeGenerator.cpp include/Planner/tool/lattice/LatticeSampler.h include/Planner/tool/lattice/LatticeCommon.h src/Planner/tool/lattice/LatticeCommon.cpp src/Planner/tool/lattice/LatticeSampler.cpp include/Planner/type/FeasibleRegion.h src/Planner/type/FeasibleRegion.cpp include/Planner/Common.h src/Planner/Common.cpp include/Planner/tool/PredictionQuerier.h src/Planner/tool/PredictionQuerier.cpp include/Planner/tool/lattice/LatticeEvaluator.h src/Planner/tool/lattice/LatticeEvaluator.cpp include/Planner/type/ConstantAccelerationCurve.h src/Planner/type/ConstantAccelerationCurve.cpp include/Planner/tool/ConstraintChecker.h src/Planner/tool/ConstraintChecker.cpp include/Planner/tool/CollisionChecker.h src/Planner/tool/CollisionChecker.cpp)
target_link_libraries(Planner
    ${catkin_LIBRARIES}
)
set_target_properties(Planner PROPERTIES OUTPUT_NAME Planner PREFIX "")
set_target_properties(Planner PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../.bin/autotune/)


#pragma nox_end()
#################################
## Build Plugin Inplementation ##
#################################
#pragma nox_begin("build plugin")


#pragma nox_end()
