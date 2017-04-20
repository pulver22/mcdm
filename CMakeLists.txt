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

add_executable(mcdm_online_exploration main_new.cpp) 
target_link_libraries(mcdm_online_exploration lib  -lboost_unit_test_framework)

install(TARGETS mcdm_online_exploration RUNTIME DESTINATION bin)
add_subdirectory(test)

#c++11
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -g  -ftest-coverage -fprofile-arcs")
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
