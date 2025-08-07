cmake_minimum_required(VERSION 3.8)
project(mujoco_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## mujoco相关配置
set(MUJOCO_PATH "/home/xiatenghui/mujoco") 
# 手动包含头文件目录
include_directories(${MUJOCO_PATH}/include)
# 手动链接库目录
link_directories(${MUJOCO_PATH}/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(ur5e_control src/ur5e_control.cpp)

# 处理非ROS包依赖
target_link_libraries(ur5e_control
  libmujoco.so
  glfw
  )

# 处理ROS包依赖
ament_target_dependencies(ur5e_control 
rclcpp
sensor_msgs)

install(TARGETS
ur5e_control
DESTINATION lib/${PROJECT_NAME})

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

