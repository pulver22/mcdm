cmake_minimum_required(VERSION 2.6)
project(mcdm_online_exploration_ros)

include_directories( . )

find_package(Boost REQUIRED)

# ${Boost_INCLUDE_DIRS}
# ${Boost_LIBRARIES}

add_library(lib pose.cpp pose.h edge.h map.h map.cpp newray.cpp newray.h ray.cpp ray.h graphpose.cpp graphpose.h mcdmfunction.cpp mcdmfunction.h 
map.cpp map.h evaluationrecords.cpp evaluationrecords.h Criteria/sensingtimecriterion.cpp Criteria/sensingtimecriterion.h
Criteria/informationgaincriterion.cpp Criteria/informationgaincriterion.h Criteria/mcdmweightreader.cpp Criteria/mcdmweightreader.h Criteria/traveldistancecriterion.cpp Criteria/traveldistancecriterion.h Criteria/weightmatrix.cpp 
Criteria/weightmatrix.h Criteria/criterioncomparator.h Criteria/criterioncomparator.cpp 
Criteria/criterion.h Criteria/criterion.cpp )

add_executable(mcdm_online_exploration_ros main_new.cpp) 
target_link_libraries(mcdm_online_exploration_ros lib  -lboost_unit_test_framework)

install(TARGETS mcdm_online_exploration_ros RUNTIME DESTINATION bin)
add_subdirectory(test)

#c++11
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -g  -ftest-coverage -fprofile-arcs")
add_subdirectory(Maps)
