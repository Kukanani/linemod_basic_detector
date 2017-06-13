cmake_minimum_required(VERSION 3.5)
project(linemod_basic_detector)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

################################################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(GLUT REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(ork_renderer REQUIRED)

################################################################################

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/linemod.cpp src/linemod_icp.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "OpenCV"
  "ork_renderer"
)

add_executable(linemod_node src/linemod_node.cpp)
ament_target_dependencies(linemod_node
  "rclcpp"
  "sensor_msgs"
  "vision_msgs"
  "OpenCV"
  "${PROJECT_NAME}"
  "ork_renderer"
  "tf2"
  "tf2_geometry_msgs"
)
target_link_libraries(linemod_node ${PROJECT_NAME})

add_executable(linemod_train_standalone_kinect
  src/linemod_train_standalone_kinect.cpp)
ament_target_dependencies(linemod_train_standalone_kinect
  "OpenCV"
  "Boost"
)

add_executable(linemod_train_virtual src/linemod_train_virtual.cpp)
ament_target_dependencies(linemod_train_virtual
  "OpenCV"
  "Boost"
  "ork_renderer"
)
target_link_libraries(linemod_train_virtual ${ork_renderer_LIBRARIES})

################################################################################

install(
  TARGETS
    linemod_node
    linemod_train_standalone_kinect
    linemod_train_virtual
    linemod_basic_detector
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

################################################################################

ament_export_libraries(linemod_basic_detector)
ament_export_include_directories(include)
ament_package()
