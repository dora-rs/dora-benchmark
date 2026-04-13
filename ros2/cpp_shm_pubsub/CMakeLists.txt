cmake_minimum_required(VERSION 3.8)
project(cpp_shm_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(iceoryx_posh REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bench8.msg"
  "msg/Bench40K.msg"
  "msg/Bench400K.msg"
  "msg/Bench4M.msg"
)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(talker "${cpp_typesupport_target}")

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp)
target_link_libraries(listener "${cpp_typesupport_target}")

add_executable(iox_talker src/iox_talker.cpp)
target_link_libraries(iox_talker iceoryx_posh::iceoryx_posh)

add_executable(iox_listener src/iox_listener.cpp)
target_link_libraries(iox_listener iceoryx_posh::iceoryx_posh)

install(TARGETS talker listener iox_talker iox_listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
