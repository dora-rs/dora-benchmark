cmake_minimum_required(VERSION 3.17)
project(dora-cuda-latency-cpp)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -O2)

# Prebuilt dora C++ libraries.
set(DORA_CPP_DIR "/tmp/dora-cpp/dora-cpp-libraries-x86_64-unknown-linux-gnu")

find_package(Arrow REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_executable(receiver
    receiver.cc
    ${DORA_CPP_DIR}/src/dora-node-api.cc
)

target_include_directories(receiver PRIVATE
    ${DORA_CPP_DIR}/include
)

target_link_libraries(receiver
    ${DORA_CPP_DIR}/lib/libdora_node_api_cxx.a
    Arrow::arrow_shared
    CUDA::cudart
    pthread
    dl
    rt
)
