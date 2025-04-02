cmake_minimum_required(VERSION 3.0.2)
project(command)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# ARM NEON flags
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  message("enabling ARM neon optimizations")
endif()

# flags for speed (should already be enabled by default)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3 -march=native")
#set(ACADO_LIBRARIES )
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf2_eigen
  mavros_msgs
 #tf2_geometry_msgs
  state_msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime 
)

add_executable(command_pub 
              src/command_pub.cpp)
target_include_directories(command_pub PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  )
target_link_libraries(command_pub ${catkin_LIBRARIES})

add_executable(command_pub_dis 
              src/command_pub_dis.cpp)
target_include_directories(command_pub_dis PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  )
target_link_libraries(command_pub_dis ${catkin_LIBRARIES})

add_executable(test_p src/test2.cpp)
target_include_directories(test_p  PUBLIC
  include 
  ${EIGEN3_INCLUDE_DIR}
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  )
target_link_libraries(test_p  ${catkin_LIBRARIES})

add_executable(pub_rc src/pub_rc.cpp)
target_include_directories(pub_rc  PUBLIC
  include 
  ${EIGEN3_INCLUDE_DIR}
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  )
target_link_libraries(pub_rc  ${catkin_LIBRARIES})

add_executable(command_pub_follower src/command_pub_follower.cpp)
target_include_directories(command_pub_follower  PUBLIC
  include 
  ${EIGEN3_INCLUDE_DIR}
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  )
target_link_libraries(command_pub_follower  ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
add_executable(vision_estimator src/vision_estimator.cpp)
target_include_directories(vision_estimator  PUBLIC
  include 
  ${EIGEN3_INCLUDE_DIR}
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  )
target_link_libraries(vision_estimator  ${catkin_LIBRARIES})
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)