cmake_minimum_required(VERSION 3.5)
project(Drost-PPF)

set(CMAKE_CXX_STANDARD 14)


find_package(PCL 1.8 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})

link_directories($PCL_LIBRARY_DIRS)

add_definitions(${PCL_DEFINITIONS})


add_executable(test main.cpp Drost_PPF.cpp Drost_PPF.h Hashmap.cpp Hashmap.h PPFRegistration.cpp PPFRegistration.h PPFEstimation.cpp PPFEstimation.h)

target_link_libraries (test ${PCL_LIBRARIES})

install(TARGETS test RUNTIME DESTINATION bin)

