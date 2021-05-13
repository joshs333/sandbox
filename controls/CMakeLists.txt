project (controls)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(
    /usr/include/python2.7
    /usr/include/eigen3
    include
)


add_library(jmath
    src/jmath/time_range.cpp
)

add_library(jcontrols
    src/jcontrols/models/pendulum.cpp
    src/jcontrols/models/bicycle_model.cpp
    src/jcontrols/models/bicycle_model2.cpp
    src/jcontrols/models/bicycle_model3.cpp
)
target_link_libraries(jcontrols
    jmath
)

add_executable(main src/main.cpp)
# add_executable(pen_tests src/pendulum_testing.cpp)
# add_executable(test src/test.cpp)

target_link_libraries(main
    jcontrols
    jmath
    python2.7
)
# target_link_libraries(pen_tests
#     jcontrols
#     jmath
# )
# target_link_libraries(test
#     jcontrols
#     jmath
# )