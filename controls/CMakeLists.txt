project (controls)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(
    /usr/local/include/eigen3
    include
)


add_library(jmath
    src/jmath/time_range.cpp
)

add_library(jcontrols
    src/jcontrols/models/pendulum.cpp
)
target_link_libraries(jcontrols
    jmath
)

add_executable(main src/main.cpp)
add_executable(test src/test.cpp)

target_link_libraries(main
    jcontrols
    jmath
)
target_link_libraries(test
    jcontrols
    jmath
)