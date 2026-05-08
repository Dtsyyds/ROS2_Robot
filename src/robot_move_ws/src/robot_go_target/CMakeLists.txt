cmake_minimum_required(VERSION 3.8)
project(robot_go_target)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(robot_gotarget_node 
  src/robot_gotarget_node.cpp 
  src/universal_robot_driver.cpp
  src/factory/robot_factory.cpp
  src/impl/agv/agv_robot_move.cpp
  src/impl/air/air_robot_move.cpp
  src/impl/duct/duct_robot_move.cpp
  src/impl/mag/mag_robot_move.cpp
  src/util/serial_port_helper.cpp
  )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)                
find_package(tf2_geometry_msgs REQUIRED)  
find_package(nlohmann_json REQUIRED)

include_directories(include)
ament_target_dependencies(robot_gotarget_node 
rclcpp 
geometry_msgs 
std_msgs tf2 
tf2_geometry_msgs
nlohmann_json
)#连接库，载入头文件路径

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

install(TARGETS 
  robot_gotarget_node
  DESTINATION lib/${PROJECT_NAME}  
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}  
)

ament_package()
