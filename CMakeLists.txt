cmake_minimum_required(VERSION 2.8.3)
project(gpd_interface)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    message_generation
    image_geometry
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    moveit_visual_tools
    robot_helpers
    cvx_util
    clopema_robot
    camera_helpers
    gpd
    basic_next_best_view
    radioroso_gripper_control
    certh_pickup
)

add_service_files(
  FILES
  getPointCloud.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        actionlib
        message_runtime
        std_msgs
        sensor_msgs
        moveit_core
        moveit_ros_planning_interface
        moveit_msgs
        moveit_visual_tools
	robot_helpers
	cvx_util
	clopema_robot
        certh_pickup
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

ADD_DEFINITIONS( -std=c++11 )

add_library(construct_point_cloud
        src/construct_point_cloud.cpp
)
target_link_libraries(construct_point_cloud ${catkin_LIBRARIES})


add_executable(move_to_nbv src/move_to_nbv_node.cpp src/move_to_nbv.cpp)
target_link_libraries(move_to_nbv construct_point_cloud ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})

add_executable(post_point_cloud src/post_point_cloud_node.cpp)
target_link_libraries(post_point_cloud construct_point_cloud ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})

add_executable(grasp_unknown_object src/grasp_unknown_object_node.cpp src/grasp_unknown_object.cpp)
target_link_libraries(grasp_unknown_object ${catkin_LIBRARIES})

add_executable(test_point_cloud_merge test/test_point_cloud_merge.cpp)
target_link_libraries(test_point_cloud_merge ${catkin_LIBRARIES})

add_executable(gpd_node src/gpd_node.cpp src/move_to_nbv.cpp src/grasp_unknown_object.cpp)
target_link_libraries(gpd_node construct_point_cloud ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})



