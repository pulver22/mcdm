cmake_minimum_required(VERSION 2.6)
project(mcdm_onlin_exploration_ros)

add_executable(mcdm_onlin_exploration_ros main.cpp)

install(TARGETS mcdm_onlin_exploration_ros RUNTIME DESTINATION bin)

