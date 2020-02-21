cmake_minimum_required(VERSION 2.8.3)
project(cape_ros)

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
  message_filters
  jsk_recognition_msgs
  jsk_topic_tools
  image_geometry
)

include_directories(include ${EIGEN3_INCLUDE_DIR})

generate_dynamic_reconfigure_options(
  cfg/Cape.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/cape_nodelet.cpp
	src/CAPE.cpp
	src/PlaneSeg.cpp
	src/CylinderSeg.cpp
	src/Histogram.cpp)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS} ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

# download and install sample data
add_custom_target(${PROJECT_NAME}_install_sample_data
  ALL COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py)
