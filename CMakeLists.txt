cmake_minimum_required(VERSION 3.1.3)
project(collision_detection_fcl_voxel)
set(MOVEIT_LIB_NAME moveit_collision_detection_fcl_voxel)
list(APPEND CMAKE_PREFIX_PATH "/usr/local/libgvl")

find_package(catkin REQUIRED
  COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  xmlrpcpp
)

find_package(CUDA REQUIRED)
find_package(icl_core REQUIRED )
find_package(gpu_voxels REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(fcl QUIET)
if (fcl_FOUND)
  set(LIBFCL_LIBRARIES ${FCL_LIBRARIES})
  set(LIBFCL_INCLUDE_DIRS ${FCL_INCLUDE_DIRS})
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBFCL_PC REQUIRED fcl)
  set(LIBFCL_INCLUDE_DIRS ${LIBFCL_PC_INCLUDE_DIRS})
  # find *absolute* paths to LIBFCL_LIBRARIES
  set(LIBFCL_LIBRARIES)
  foreach(_lib ${LIBFCL_PC_LIBRARIES})
    find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
    list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
  endforeach()
endif()

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(octomap REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

message(STATUS "GVL include dirs are: ${gpu_voxels_INCLUDE_DIRS}")
message(STATUS "icl include dirs are: ${icl_core_INCLUDE_DIRS}")

set(THIS_PACKAGE_INCLUDE_DIRS
    #doc/interactivity/include
    include
)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES
    moveit_collision_detection_fcl_voxel
  CATKIN_DEPENDS
    eigen_stl_containers
    geometric_shapes
    geometry_msgs
    kdl_parser
    #srdfdom
    #tf2_eigen
    #tf2_geometry_msgs
    moveit_core
    #moveit_visual_tools
    #moveit_ros_planning_interface
    #interactive_markers
  DEPENDS
    Boost
    EIGEN3
    LIBFCL
    OCTOMAP
    urdfdom
    urdfdom_headers
)
add_definitions(
  ${icl_core_DEFINITIONS}
  ${gpu_voxels_DEFINITIONS}
)
message("================ icl_core_DEFINITIONS ${icl_core_DEFINITIONS} ===================")
message("================ icl_core_LIBRARIES  ${icl_core_LIBRARIES} ================")


include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS} 
  PUBLIC ${catkin_INCLUDE_DIRS} 
  PUBLIC ${Boost_INCLUDE_DIR} 
  PUBLIC ${EIGEN3_INCLUDE_DIRS}
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
  PUBLIC ${icl_core_INCLUDE_DIRS}
  PUBLIC ${gpu_voxels_INCLUDE_DIRS}
  PUBLIC ${CUDA_INCLUDE_DIRS}
)

add_library(${MOVEIT_LIB_NAME}
  src/collision_common.cpp
  src/collision_robot_fcl.cpp
  src/collision_world_fcl.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_compile_definitions(${MOVEIT_LIB_NAME} PRIVATE _SYSTEM_POSIX_=1)

target_link_libraries(
  ${MOVEIT_LIB_NAME} 
  LINK_PUBLIC ${catkin_LIBRARIES} 
  LINK_PUBLIC ${urdfdom_LIBRARIES} 
  LINK_PUBLIC ${urdfdom_headers_LIBRARIES} 
  LINK_PUBLIC ${LIBFCL_LIBRARIES} 
  LINK_PUBLIC ${Boost_LIBRARIES}
  LINK_PUBLIC ${icl_core_LIBRARIES}
  LINK_PUBLIC ${gpu_voxels_LIBRARIES}
  LINK_PUBLIC ${CUDA_LIBRARIES}
)

add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

add_library(collision_detector_fcl_voxel_plugin src/collision_detector_fcl_plugin_loader.cpp)
set_target_properties(collision_detector_fcl_voxel_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  collision_detector_fcl_voxel_plugin 
  ${catkin_LIBRARIES} 
  ${MOVEIT_LIB_NAME}
  ${Boost_SYSTEM_LIBRARY} 
  ${icl_core_LIBRARIES} 
  ${gpu_voxels_LIBRARIES} 
  ${CUDA_LIBRARIES}
)


install(TARGETS ${MOVEIT_LIB_NAME} collision_detector_fcl_voxel_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES collision_detector_fcl_voxel_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_fcl_collision_detection test/test_fcl_collision_detection.cpp)
  target_link_libraries(test_fcl_collision_detection moveit_test_utils ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})
  # TODO: remove if transition to gtest's new API TYPED_TEST_SUITE_P is finished
  target_compile_options(test_fcl_collision_detection PRIVATE -Wno-deprecated-declarations)

endif()
