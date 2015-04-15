cmake_minimum_required(VERSION 2.6)
project(mcdm_online_exploration_ros)

add_executable(mcdm_online_exploration_ros main.cpp ) 
# target_link_libraries(exec -lboost_unit_test_framework)

install(TARGETS mcdm_online_exploration_ros RUNTIME DESTINATION bin)