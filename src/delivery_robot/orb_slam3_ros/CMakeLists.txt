cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV 4.6 REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(Pangolin REQUIRED)

# ORB-SLAM3 library (built separately in /home/pi/third_party/ORB_SLAM3)
set(ORB_SLAM3_DIR "/home/pi/third_party/ORB_SLAM3")
set(ORB_SLAM3_INCLUDE_DIRS
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/g2o
)
set(ORB_SLAM3_LIBS
  "${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so"
  "${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so"
  "${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so"
)

add_executable(orb_slam3_node src/orb_slam3_node.cpp)

target_include_directories(orb_slam3_node PRIVATE
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(orb_slam3_node
  ${ORB_SLAM3_LIBS}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Eigen3::Eigen
  -lboost_serialization
  -lcrypto
)

ament_target_dependencies(orb_slam3_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  tf2
  image_transport
)

install(TARGETS orb_slam3_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
