cmake_minimum_required(VERSION 3.8)
project(avg428_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找必要套件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)

# 建立執行檔
add_executable(vehicle_interface src/vehicle_interface.cpp)

# 連結相依套件
ament_target_dependencies(vehicle_interface
  rclcpp
  autoware_vehicle_msgs
  autoware_control_msgs
)

# 安裝執行檔
install(TARGETS
  vehicle_interface
  DESTINATION lib/${PROJECT_NAME}
)

# 安裝 launch 檔案
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
