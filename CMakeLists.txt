cmake_minimum_required(VERSION 2.6)
project(mcdm_online_exploration_ros)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}

)

find_package(Boost REQUIRED)

# ${Boost_INCLUDE_DIRS}
# ${Boost_LIBRARIES}

add_library(lib pose.cpp PathFinding/astar.cpp PathFinding/node.cpp map.cpp newray.cpp mcdmfunction.cpp
map.cpp evaluationrecords.cpp Criteria/sensingtimecriterion.cpp Criteria/informationgaincriterion.cpp Criteria/mcdmweightreader.cpp Criteria/traveldistancecriterion.cpp Criteria/weightmatrix.cpp Criteria/criterioncomparator.cpp Criteria/criterion.cpp  )

#Find all the files link them to the executable (for qtcreator)
FILE(GLOB_RECURSE HEADER_FILES include *.h *.cpp)
add_executable(mcdm_online_exploration main_new.cpp  ${HEADER_FILES})

target_link_libraries(mcdm_online_exploration lib  -lboost_unit_test_framework)

install(TARGETS mcdm_online_exploration RUNTIME DESTINATION bin)
add_subdirectory(test)

#c++11
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -g  -ftest-coverage -fprofile-arcs")
