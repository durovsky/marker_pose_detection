cmake_minimum_required(VERSION 2.8.3)
project(viewpoint_estimation)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  message_generation
  image_transport 
  cv_bridge
  tf
  pal_vision_segmentation
  aruco
  aruco_msgs
)

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/
                    )

SET(SOURCES ${PROJECT_SOURCE_DIR}/src/viewpoint_estimation.cpp
            ${PROJECT_SOURCE_DIR}/src/viewpoint_estimation_lib.cpp
            )

SET(HEADERS ${PROJECT_SOURCE_DIR}/include/viewpoint_estimation_lib.h
            )

catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp 
  LIBRARIES
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES}
)

