cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(perception REQUIRED)

# 이미지 처리를 위한 필수 패키지 추가
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# 실행 파일 생성
add_executable(planning_node src/planning_node.cpp)

# 의존성 설정 (image_transport와 cv_bridge 추가)
ament_target_dependencies(planning_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  nav_msgs 
  std_msgs 
  visualization_msgs 
  perception
  image_transport
  cv_bridge
)

# 설치 경로 설정
install(TARGETS
  planning_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
