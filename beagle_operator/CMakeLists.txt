cmake_minimum_required(VERSION 3.5)
project(beagle_operator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_PREFIX_PATH /home/hyedo/robot_ws/install/beagle_msgs ${CMAKE_PREFIX_PATH})


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(beagle_msgs REQUIRED)

include_directories( include
    ${beagle_msgs_INCLUDE_DIRS}
  )

add_executable(beagle_operator src/beagle_operator.cpp)
ament_target_dependencies(beagle_operator
  rclcpp
  std_msgs
  beagle_msgs
)

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


# .hpp파일
install(TARGETS
  beagle_operator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
