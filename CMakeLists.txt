cmake_minimum_required(VERSION 3.8)
project(tnvcam_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
pkg_check_modules(GSTREAMERMM REQUIRED gstreamermm-1.0)


include_directories(include
 ${GSTREAMERMM_INCLUDE_DIRS}
 )


add_executable(nvv4l2_node src/nvv4l2_node.cpp src/camera_pipeline.cpp)
ament_target_dependencies(nvv4l2_node
  rclcpp
  sensor_msgs
)

target_link_libraries(nvv4l2_node
  ${GSTREAMERMM_LIBRARIES}
)

# 실행 파일 설치
install(TARGETS
  nvv4l2_node
  DESTINATION lib/${PROJECT_NAME}
)

# 설치할 launch 파일을 명시
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
