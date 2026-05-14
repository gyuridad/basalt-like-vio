cmake_minimum_required(VERSION 3.8)
project(basalt_like_vio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# include directory
include_directories(
  include
)

# ================================
# Isaac IMU Preintegration Node
# ================================

# -- 실행 파일 만들기 --
# src/isaac_imu_preintegration_node.cpp 파일을 컴파일해서
# isaac_imu_preintegration_node라는 실행 파일을 만들어라.
add_executable(isaac_imu_preintegration_node
  src/isaac_imu_preintegration_node.cpp
)

# -- include 폴더 연결 --
# 여기서 PRIVATE는 쉽게 말하면:
# 이 include 경로는 이 실행 파일을 빌드할 때만 사용한다는 의미야.
target_include_directories(isaac_imu_preintegration_node
  PRIVATE
    include
)

# -- ROS2 의존성 연결 --
ament_target_dependencies(isaac_imu_preintegration_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# -- Eigen 연결 --
target_link_libraries(isaac_imu_preintegration_node
  Eigen3::Eigen
)

# -- 설치 설정 --
# 이 부분은 빌드한 실행 파일을 ROS2가 찾을 수 있는 위치에 설치하는 설정이야.
install(TARGETS 
  isaac_imu_preintegration_node
  DESTINATION lib/${PROJECT_NAME}
)

# ================================
# Isaac IMU Preintegration LDA Diagnostic Node
# ================================

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/isaac_imu_preintegration_lda_node.cpp")
  add_executable(isaac_imu_preintegration_lda_node
    src/isaac_imu_preintegration_lda_node.cpp
  )

  target_include_directories(isaac_imu_preintegration_lda_node
    PRIVATE
      include
  )

  ament_target_dependencies(isaac_imu_preintegration_lda_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
  )

  target_link_libraries(isaac_imu_preintegration_lda_node
    Eigen3::Eigen
  )

  install(TARGETS
    isaac_imu_preintegration_lda_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(PROGRAMS
  scripts/isaac_imu_lda_debug_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
