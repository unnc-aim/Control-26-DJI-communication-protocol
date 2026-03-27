cmake_minimum_required(VERSION 3.8)
project(dji_referee_protocol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==================== 依赖查找 ====================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# ==================== 消息生成 ====================
# 使用通配符获取所有消息文件
file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)

# ==================== Python安装 ====================
install(DIRECTORY
  dji_referee_protocol/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# 安装Python可执行脚本
install(PROGRAMS
  scripts/referee_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

# ==================== 数据文件安装 ====================
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ==================== 测试 ====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
