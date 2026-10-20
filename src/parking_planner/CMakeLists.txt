cmake_minimum_required(VERSION 3.8)
project(parking_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# parking_core
add_library(parking_core STATIC
  src/core/core_smoke.cpp
  src/core/geometry/angle.cpp
  src/core/geometry/vec2d.cpp
  src/core/geometry/transform2d.cpp
)

target_include_directories(parking_core PUBLIC
  include
)

target_compile_features(parking_core PUBLIC cxx_std_17)

# runtime 
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(parking_runtime STATIC
  src/runtime/parking_node.cpp
  src/visualization/visualizer.cpp
)

target_include_directories(parking_runtime 
PUBLIC
  include
)

target_compile_features(parking_runtime PUBLIC cxx_std_17)

ament_target_dependencies(parking_runtime 
PUBLIC
  rclcpp
  visualization_msgs
  geometry_msgs
)

target_link_libraries(parking_runtime 
PRIVATE
  parking_core
)


# main
add_executable(parking_node
  src/main.cpp
)

target_link_libraries(parking_node PRIVATE
  parking_runtime
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_core_smoke
    test/core/test_core_smoke.cpp
  )

  target_link_libraries(test_core_smoke
    parking_core
  )

  ament_add_gtest(test_angle
    test/core/geometry/test_angle.cpp
  )

  target_link_libraries(test_angle
    parking_core
  )

  ament_add_gtest(test_vec2d
    test/core/geometry/test_vec2d.cpp
  )

  target_link_libraries(test_vec2d
    parking_core
  )

  ament_add_gtest(test_transform2d
    test/core/geometry/test_transform2d.cpp
  )

  target_link_libraries(test_transform2d
    parking_core
  )

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

install(TARGETS parking_core parking_runtime
  ARCHIVE DESTINATION lib
)

install(TARGETS parking_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
