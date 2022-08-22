cmake_minimum_required(VERSION 3.5)
project(Drost-PPF)

set(CMAKE_CXX_STANDARD 17)


find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(${PCL_INCLUDE_DIRS})

link_directories($PCL_LIBRARY_DIRS)

add_definitions(${PCL_DEFINITIONS})

add_executable(Drost_PPF main.cpp Drost_PPF.cpp Drost_PPF.h PPFRegistration.cpp PPFRegistration.h PPFEstimation.cpp PPFEstimation.h Hashmap.hpp)

target_link_libraries (Drost_PPF ${PCL_LIBRARIES})

install(TARGETS Drost_PPF RUNTIME DESTINATION bin)

