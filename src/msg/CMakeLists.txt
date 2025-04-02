# cmake_minimum_required(VERSION 3.0.2)
# project(state_msg)

# find_package(catkin REQUIRED COMPONENTS
#     message_generation  
#     roscpp
#     geometry_msgs
#     actionlib_msgs
#     sensor_msgs
#     nav_msgs
#     std_msgs
#     std_srvs
#     tf2_ros
#     tf2_eigen
#     mavros_msgs
# )

# add_message_files(
#   DIRECTORY msg
#   FILES
#   form.msg

# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs 
#   sensor_msgs
#   std_msgs
# )

# catkin_package(
#     CATKIN_DEPENDS
#     message_runtime
#     actionlib_msgs
#     geometry_msgs
#     sensor_msgs
#     message_runtime
#     std_msgs
# )
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )


# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )