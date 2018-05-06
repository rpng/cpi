cmake_minimum_required(VERSION 2.8.8)

# Project name
project(cpi_compare)

# Include our cmake files
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

# Find catkin (the ROS build system)
find_package(catkin REQUIRED COMPONENTS roscpp tf pcl_ros std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs pcl_conversions cpi_comm)

# Include libraries
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(GTSAM REQUIRED) # source: https://bitbucket.org/gtborg/gtsam/
find_package(MKL REQUIRED) # intel MKL library

# Describe catkin project
catkin_package(
    CATKIN_DEPENDS roscpp tf pcl_ros std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs pcl_conversions cpi_comm
    INCLUDE_DIRS src
)

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall")

# Include all thirdparty source files in build
# NOTE: we need Eigen since GTSAM is picky about what Eigen it will use
file(GLOB_RECURSE external_headers external/*.h external/*.hpp)
file(GLOB_RECURSE external_sources external/*.cpp)

# Include our header files
include_directories(
    src
    external
    #${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${MLK_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${MKL_LIBRARIES}
    gtsam
    gtsam_unstable
    ${catkin_LIBRARIES}
)



##################################################
# Make binary files!
##################################################
add_executable(estimator
        src/main_estimator.cpp
        src/gtsam/JPLNavState.cpp
        src/gtsam/JPLNavStatePrior.cpp
        src/gtsam/ImuFactorCPIv1.cpp
        src/gtsam/ImuFactorCPIv2.cpp
        src/gtsam/JPLImageUVFactor.cpp
        src/gtsam/InvAnchorFactor.cpp
        src/gtsam/InvUVFactor.cpp
        src/solvers/GraphSolver.cpp
        src/solvers/GraphSolver_IMU.cpp
        src/solvers/GraphSolver_FEAT.cpp
        src/solvers/FeatureInitializer.cpp
        src/cpi/CpiBase.h
        src/cpi/CpiV1.h
        src/cpi/CpiV2.h
        src/sim/SimulationLoader.cpp
        src/sim/SimParser.h
        ${external_sources}
)
target_link_libraries(estimator ${thirdparty_libraries})







