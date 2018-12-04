cmake_minimum_required(VERSION 2.8.3)
project(autotune)

#####################
## Compile Setting ##
#####################

add_compile_options(-std=c++17)

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
        traffic_light
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
add_executable(autotune_Planner
        include/Planner/.PlannerModule.h src/Planner/.PlannerModule.cpp
        include/Planner/Planner.h src/Planner/Planner.cpp
        include/Planner/type/ReferenceLine.h src/Planner/type/ReferenceLine.cpp
        include/Planner/tool/TrajectoryStitcher.h src/Planner/tool/TrajectoryStitcher.cpp
        include/Planner/tool/DecisionMaker.h src/Planner/tool/DecisionMaker.cpp
        include/Planner/impl/PlannerBase.h src/Planner/impl/PlannerBase.cpp
        include/Planner/impl/LatticePlanner.h src/Planner/impl/LatticePlanner.cpp
        include/Planner/type/SLTBoundary.h include/Planner/type/STGraph.h src/Planner/type/STGraph.cpp
        include/Planner/tool/lattice/LatticeGenerator.h src/Planner/tool/lattice/LatticeGenerator.cpp
        include/Planner/tool/lattice/LatticeSampler.h  src/Planner/tool/lattice/LatticeSampler.cpp
        include/Planner/tool/lattice/LatticeCommon.h src/Planner/tool/lattice/LatticeCommon.cpp
        include/Planner/type/FeasibleRegion.h src/Planner/type/FeasibleRegion.cpp
        include/Planner/PlannerConfig.h
        include/Planner/tool/PredictionQuerier.h src/Planner/tool/PredictionQuerier.cpp
        include/Planner/tool/lattice/LatticeEvaluator.h src/Planner/tool/lattice/LatticeEvaluator.cpp
        include/Planner/type/PiecewiseAccelerationCurve.h src/Planner/type/PiecewiseAccelerationCurve.cpp
        include/Planner/tool/ConstraintChecker.h src/Planner/tool/ConstraintChecker.cpp
        include/Planner/tool/CollisionChecker.h src/Planner/tool/CollisionChecker.cpp
        include/Planner/type/ConstantJerkCurve.h src/Planner/type/ConstantJerkCurve.cpp
        include/Planner/type/PiecewiseJerkCurve.h src/Planner/type/PiecewiseJerkCurve.cpp src/Planner/rule/StopLine.cpp
        include/Planner/rule/StopLine.h include/Planner/rule/Passable.h src/Planner/rule/Passable.cpp include/Tracking/tool/Guardian.h src/Planner/type/ConstantAccelerationCurve.cpp include/Planner/tool/PiecewiseBrakingTrajectoryGenerator.h src/Planner/tool/PiecewiseBrakingTrajectoryGenerator.cpp)
target_link_libraries(autotune_Planner
    ${catkin_LIBRARIES}
)
set_target_properties(autotune_Planner PROPERTIES OUTPUT_NAME Planner PREFIX "")
set_target_properties(autotune_Planner PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../.bin/autotune/)

add_executable(autotune_LoMap
    include/LoMap/.LoMapModule.h src/LoMap/.LoMapModule.cpp
    include/LoMap/LoMap.h src/LoMap/LoMap.cpp
        include/LoMap/tool/SceneMaintainer.h src/LoMap/tool/SceneMaintainer.cpp
        include/LoMap/tool/PoseEstimator.h src/LoMap/tool/PoseEstimator.cpp
        include/LoMap/tool/StaticSceneUpdater.h src/LoMap/tool/StaticSceneUpdater.cpp
        include/LoMap/tool/IDProvider.h src/LoMap/tool/IDProvider.cpp
        include/LoMap/tool/SceneObjectCreator.h
        include/LoMap/tool/DynamicSceneUpdater.h src/LoMap/tool/DynamicSceneUpdater.cpp
        include/LoMap/tool/SceneOverlapBuilder.h src/LoMap/tool/SceneOverlapBuilder.cpp
        include/LoMap/LoMapConfig.h include/LoMap/tool/GuideLineBuilder.h include/LoMap/type/ControlLine.h src/LoMap/tool/GuideLineBuilder.cpp include/LoMap/pattern/SupplyChain.h include/LoMap/provider/GuideLineProvider.h src/LoMap/provider/GuideLineProvider.cpp include/LoMap/type/Material.h include/LoMap/tool/SceneGenerator.h include/LoMap/provider/MapProvider.h src/LoMap/provider/MapProvider.cpp src/LoMap/tool/SceneGenerator.cpp include/LoMap/type/MD5.h include/LoMap/provider/TrafficLightProvider.h include/LoMap/provider/VehicleStateProvider.h src/LoMap/provider/VehicleStateProvider.cpp include/LoMap/provider/ObstacleProvider.h src/LoMap/provider/ObstacleProvider.cpp src/LoMap/provider/TrafficLightProvider.cpp include/LoMap/provider/SceneProvider.h src/LoMap/provider/SceneProvider.cpp include/LoMap/provider/StopLineProvider.h src/LoMap/provider/StopLineProvider.cpp)
target_link_libraries(autotune_LoMap
    ${catkin_LIBRARIES}
)
set_target_properties(autotune_LoMap PROPERTIES OUTPUT_NAME LoMap PREFIX "")
set_target_properties(autotune_LoMap PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../.bin/autotune/)

add_executable(autotune_FakeHDMap
    include/FakeHDMap/.FakeHDMapModule.h src/FakeHDMap/.FakeHDMapModule.cpp
    include/FakeHDMap/FakeHDMap.h src/FakeHDMap/FakeHDMap.cpp
)
target_link_libraries(autotune_FakeHDMap
    ${catkin_LIBRARIES}
)
set_target_properties(autotune_FakeHDMap PROPERTIES OUTPUT_NAME FakeHDMap PREFIX "")
set_target_properties(autotune_FakeHDMap PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../.bin/autotune/)

add_executable(autotune_Tracking
    include/Tracking/.TrackingModule.h src/Tracking/.TrackingModule.cpp
    include/Tracking/Tracking.h src/Tracking/Tracking.cpp
        include/Tracking/component/interface/LongitudinalController.h include/Tracking/component/interface/LateralController.h include/Tracking/component/SimpleLongitudinalController.h src/Tracking/component/SimpleLongitudinalController.cpp include/Tracking/component/VTF.h src/Tracking/component/VTF.cpp include/Tracking/tool/Filter.h src/Tracking/tool/Filter.cpp include/Tracking/rule/Clipping.h src/Tracking/rule/Clipping.cpp include/Tracking/rule/Lowpass.h src/Tracking/rule/Lowpass.cpp include/Tracking/rule/Limiting.h src/Tracking/rule/Limiting.cpp include/Tracking/rule/Custom.h src/Tracking/rule/Custom.cpp include/Planner/type/ConstantAccelerationCurve.h include/Tracking/component/PCPID.h src/Tracking/component/PCPID.cpp)
target_link_libraries(autotune_Tracking
    ${catkin_LIBRARIES}
)
set_target_properties(autotune_Tracking PROPERTIES OUTPUT_NAME Tracking PREFIX "")
set_target_properties(autotune_Tracking PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../.bin/autotune/)


#pragma nox_end()
#################################
## Build Plugin Inplementation ##
#################################
#pragma nox_begin("build plugin")


#pragma nox_end()
