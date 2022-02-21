cmake_minimum_required(VERSION 3.8)
project(mxr_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mxr_msgs REQUIRED)

find_package(CarCom REQUIRED)
find_package(MXObjects REQUIRED)

include_directories(./include)
include_directories(${CARCOM_INCLUDE_DIRS})
include_directories(${MXObjects_INCLUDE_DIRS})

add_executable(mxr_base_node src/mxr_base_main.cpp src/mxr_base/mxr_base_node.cpp )
ament_target_dependencies(mxr_base_node rclcpp std_msgs sensor_msgs mxr_msgs)

target_link_libraries(mxr_base_node  ${MXObjects_LIBRARIES} ${CARCOM_LIBRARIES})

install(TARGETS
  mxr_base_node
  DESTINATION lib/${PROJECT_NAME})
  


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
