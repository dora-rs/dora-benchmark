cmake_minimum_required(VERSION 3.17)
project(cuda_cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cuda_interfaces REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_executable(cuda_talker src/cuda_talker.cpp)
ament_target_dependencies(cuda_talker rclcpp std_msgs cuda_interfaces)
target_link_libraries(cuda_talker CUDA::cudart)

add_executable(cuda_listener src/cuda_listener.cpp)
ament_target_dependencies(cuda_listener rclcpp std_msgs cuda_interfaces)
target_link_libraries(cuda_listener CUDA::cudart)

install(TARGETS cuda_talker cuda_listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
