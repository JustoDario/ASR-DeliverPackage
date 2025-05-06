cmake_minimum_required(VERSION 3.8)
project(navigation_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(navigation_system_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
  rcl_interfaces
  lifecycle_msgs
  navigation_system_interfaces
  ament_index_cpp
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/navigation_system/NavigationSystem.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(navigation_system_node src/navigation_system_node.cpp)
ament_target_dependencies(navigation_system_node ${dependencies})
target_link_libraries(navigation_system_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  navigation_system_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
