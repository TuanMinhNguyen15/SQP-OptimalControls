cmake_minimum_required(VERSION 3.24)
project(SQP_Optimal_Controls)

find_package(osqp REQUIRED
             PATHS "D:\\osqp\\install\\")

find_package(osqp_interface REQUIRED
             PATHS "D:\\cmake_installs\\osqp-interface\\")

add_executable(demo ./examples/demo.cpp)
target_link_libraries(demo PRIVATE osqp_interface::osqp_interface)

add_library(pendulum STATIC ./src/pendulum.cpp)
target_link_libraries(pendulum PUBLIC osqp_interface::osqp_interface)
target_include_directories(pendulum PUBLIC ./include)

add_executable(pendulum_demo ./examples/pendulum_demo.cpp)
target_link_libraries(pendulum_demo PRIVATE pendulum)

add_executable(linear_function ./examples/linear_function.cpp)
target_link_libraries(linear_function PRIVATE osqp_interface::osqp_interface)

add_executable(nonlinear_function ./examples/nonlinear_function.cpp)
target_link_libraries(nonlinear_function PRIVATE osqp_interface::osqp_interface)

add_executable(nonlinear_function2 ./examples/nonlinear_function2.cpp)
target_link_libraries(nonlinear_function2 PRIVATE osqp_interface::osqp_interface)