cmake_minimum_required(VERSION 2.8.3)
project(mesh_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mesh_client
  mesh_msgs_conversions
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  xmlrpcpp
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(LVR2 2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)


generate_dynamic_reconfigure_options(
  cfg/MeshMap.cfg
)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES mesh_map
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    mesh_client
    mesh_msgs_conversions
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    xmlrpcpp
  DEPENDS
    LVR2
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LVR2_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mesh_map.cpp
  src/util.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

# on runtime, the linker seems to fail to find liblvr2rply.so
# which is definitely installed with lvr2 at /usr/local/lib/liblvr2rply.so
# i tried using the following to fix it, but it didn't work. 
# For now, we need to manually export usr/local/lib to the LD_LIBRARY_PATH:
#
#             export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
#
# find_library(LVR2RPLY_LIBRARY NAMES lvr2rply)
# find_library(LVR2RPLY_LIBRARY_STATIC NAMES lvr2rply_static)
# list(APPEND LVR2_LIBRARIES ${LVR2RPLY_LIBRARY} ${LVR2RPLY_LIBRARY_STATIC})
# message("==========================================")
# message("LVR2 Libraries: ${LVR2_LIBRARIES}")
# message("==========================================")

link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${LVR2_LIBRARIES}
	${LVR2_LIB_DEPENDENCIES}
  # lvr2rply
	# lvr2rply_static
  # lvr2_static
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

