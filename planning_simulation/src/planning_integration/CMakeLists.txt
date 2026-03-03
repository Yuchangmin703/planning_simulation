cmake_minimum_required(VERSION 3.8)
project(planning_integration)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(planning_node src/planning_node.cpp)
ament_target_dependencies(planning_node rclcpp geometry_msgs nav_msgs visualization_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS planning_node DESTINATION lib/${PROJECT_NAME})

# 🎯 파일이 있는 정확한 경로(하위 폴더)를 지정합니다!
install(PROGRAMS 
  planning_integration/perception_sim.py 
  planning_integration/circuit_sil_sim.py  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
