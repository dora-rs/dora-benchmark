cmake_minimum_required(VERSION 3.17)
project(dora-cuda-latency-cpp)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -O2)

# Download prebuilt dora C++ libraries from GitHub release.
set(DORA_CPP_VERSION "v0.0.0-test-pr-1611-v2" CACHE STRING "Dora C++ release tag")
set(DORA_CPP_ARCHIVE "dora-cpp-libraries-x86_64-unknown-linux-gnu.tar.gz")
set(DORA_CPP_URL "https://github.com/dora-rs/dora/releases/download/${DORA_CPP_VERSION}/${DORA_CPP_ARCHIVE}")
set(DORA_CPP_DIR "${CMAKE_CURRENT_BINARY_DIR}/dora-cpp-libraries-x86_64-unknown-linux-gnu")

if(NOT EXISTS "${DORA_CPP_DIR}")
    message(STATUS "Downloading dora C++ libraries from ${DORA_CPP_URL}")
    file(DOWNLOAD "${DORA_CPP_URL}" "${CMAKE_CURRENT_BINARY_DIR}/${DORA_CPP_ARCHIVE}"
         STATUS download_status)
    list(GET download_status 0 status_code)
    if(NOT status_code EQUAL 0)
        message(FATAL_ERROR "Failed to download dora C++ libraries: ${download_status}")
    endif()
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_BINARY_DIR}/${DORA_CPP_ARCHIVE}"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )
endif()

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
