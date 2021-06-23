project(sb_gtsam)
cmake_minimum_required(VERSION 2.8.9)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
   /usr/include/eigen3
)


add_executable(sb_gtsam_test
    src/main.cpp
)

target_link_libraries(sb_gtsam_test
    gtsam
    tbb
)