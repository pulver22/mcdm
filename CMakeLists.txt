cmake_minimum_required(VERSION 2.6)
project(mcdm_online_exploration)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED)
# ${Boost_INCLUDE_DIRS}
# ${Boost_LIBRARIES}

## mfc for grid_map ...........................................................
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include (CTest)
find_program( MEMORYCHECK_COMMAND valgrind )
set( MEMORYCHECK_COMMAND_OPTIONS "--trace-children=yes --leak-check=full" )
set( MEMORYCHECK_SUPPRESSIONS_FILE "${PROJECT_SOURCE_DIR}/valgrind_suppress.txt" )



find_package( catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_ros
  grid_map_msgs
  grid_map_cv
  cv_bridge
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

add_executable(grid_map_test grid_map_test.cpp  ${HEADER_FILES})

target_link_libraries(grid_map_test lib  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

install(TARGETS grid_map_test RUNTIME DESTINATION bin)

add_test(grid_test grid_map_test)

## ............................................................................

add_library(lib pose.cpp PathFinding/astar.cpp PathFinding/node.cpp map.cpp newray.cpp mcdmfunction.cpp evaluationrecords.cpp Criteria/sensingtimecriterion.cpp Criteria/informationgaincriterion.cpp Criteria/mcdmweightreader.cpp Criteria/traveldistancecriterion.cpp Criteria/weightmatrix.cpp Criteria/criterioncomparator.cpp Criteria/criterion.cpp  RFIDGridmap.cpp)

#Find all the files link them to the executable (for qtcreator)
FILE(GLOB_RECURSE HEADER_FILES include *.h)
add_executable(mcdm_online_exploration main_correct_astar.cpp  ${HEADER_FILES})

target_link_libraries(mcdm_online_exploration lib  -lboost_unit_test_framework  ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES})

install(TARGETS mcdm_online_exploration RUNTIME DESTINATION bin)
#add_subdirectory(test)


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
