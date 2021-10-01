cmake_minimum_required(VERSION 3.0.0)
project(RF210001 VERSION 0.1.0)

include(CTest)
enable_testing()

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(RF210001 main.cpp pose.cpp joint.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

target_link_libraries (RF210001 Eigen3::Eigen)