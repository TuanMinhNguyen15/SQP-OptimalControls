cmake_minimum_required(VERSION 3.24)
project(SQP_Optimal_Controls)

find_package(osqp REQUIRED
             PATHS "D:\\osqp\\install\\")

find_package(osqp_interface REQUIRED
             PATHS "D:\\cmake_installs\\osqp-interface\\")

add_library(pendulum STATIC ./src/pendulum.cpp)
target_link_libraries(pendulum PUBLIC osqp_interface::osqp_interface)
target_include_directories(pendulum PUBLIC ./include)

add_executable(pendulum_demo ./examples/pendulum_demo.cpp)
target_link_libraries(pendulum_demo PRIVATE pendulum)
