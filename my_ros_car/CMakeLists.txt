cmake_minimum_required(VERSION 3.20)
project(my_ros_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 安装 URDF 文件
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf/
)

# 安装 Launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# 安装 配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# 安装 地图文件
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps/
)

# 安装 世界文件
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds/
)

# 安装 Python 脚本
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts/
  USE_SOURCE_PERMISSIONS
)

# 安装 RViz 配置文件
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz/
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
