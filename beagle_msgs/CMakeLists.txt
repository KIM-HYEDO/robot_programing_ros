cmake_minimum_required(VERSION 3.5)
project(beagle_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# file(GLOB msg_files "msg/*.msg")
# file(GLOB srv_files "srv/*.srv")
# file(GLOB action_files "action/*.action")

set(msg_files
  "msg/BuzzerControl.msg"
  "msg/CameraDetection.msg"
  "msg/LidarData.msg"
  "msg/MoterControl.msg"
  "msg/Rect.msg"
  "msg/Rects.msg"
  "msg/ServoControl.msg"
)
# set(srv_files
# )
# set(action_files
# )
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # ${srv_files}
  # ${action_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
