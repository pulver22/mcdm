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

## System dependencies are found with CMake's conventions
#find_package(Eigen3 REQUIRED)
# Solution to find Eigen3 with Saucy.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()
# find_package (Eigen3 REQUIRED)# NO_MODULE)


find_package(yaml-cpp REQUIRED)

# check for OpenMP
# find_package(OpenMP)
# # if OpenMP is available
# if(OPENMP_FOUND)
#     message(WARNING "OpenMP FOUND!!")
# # if OpenMP is not available
# else()
#     message(WARNING "OpenMP not found. Cannot run benchmarks without it")
# endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  # ${OPENMP_INCLUDES}
)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

find_package(Boost REQUIRED COMPONENTS random)

find_library(RT_LIBRARY NAMES rt librt)

# ............................................................................

#add_executable(test_log_model test_log_model.cpp  ${HEADER_FILES})

#target_link_libraries(test_log_model lib  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

#install(TARGETS test_log_model RUNTIME DESTINATION bin)

#add_executable(test_gaussian_kernel test_gaussian_kernel.cpp  ${HEADER_FILES})

#target_link_libraries(test_gaussian_kernel lib  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

#install(TARGETS test_gaussian_kernel RUNTIME DESTINATION bin)


# add_executable(test_kalman kalman-cpp/kalman-test.cpp  ${HEADER_FILES})

# target_link_libraries(test_kalman lib  ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

# install(TARGETS test_kalman RUNTIME DESTINATION bin)

# ............................................................................

add_executable(grid_map_test grid_map_test.cpp  ${HEADER_FILES})

target_link_libraries(grid_map_test lib  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

install(TARGETS grid_map_test RUNTIME DESTINATION bin)

add_test(grid_test grid_map_test)

## ............................................................................
FILE(GLOB_RECURSE ALGLIB_FILES include *.cpp)

add_library(lib RadarModel.cpp pose.cpp PathFinding/astar.cpp PathFinding/node.cpp map.cpp newray.cpp mcdmfunction.cpp evaluationrecords.cpp
       Criteria/mcdmweightreader.cpp Criteria/batterystatuscriterion.cpp Criteria/sensingtimecriterion.cpp Criteria/informationgaincriterion.cpp
        Criteria/traveldistancecriterion.cpp Criteria/weightmatrix.cpp Criteria/criterioncomparator.cpp Criteria/criterion.cpp
        RFIDGridmap.cpp Criteria/RFIDCriterion.cpp utils.cpp
       # kalman-cpp/kalman.cpp play_with_eigen.cpp
       # alglib/src/interpolation.cpp alglib/src/ap.cpp alglib/src/optimization.cpp alglib/src/alglibinternal.cpp alglib/src/linalg.cpp
       # alglib/src/alglibmisc.cpp alglib/src/solvers.cpp alglib/src/integration.cpp alglib/src/specialfunctions.cpp 
)


#Find all the files link them to the executable (for qtcreator)
FILE(GLOB_RECURSE HEADER_FILES include *.h)
# FILE(GLOB_RECURSE SOURCE_FILES include *.cpp)

add_executable(mcdm_online_exploration mcdm.cpp  ${HEADER_FILES})
# include the OpenMP compile flags as well
# target_compile_options(mcdm_online_exploration PRIVATE)# ${OpenMP_CXX_FLAGS})
# add the OpenMP libraries for linking
if (${RT_LIBRARY-NOTFOUND})
    message(WARNING "rt library not found. Linking may fail.")
  else()
    target_link_libraries(mcdm_online_exploration ${RT_LIBRARY})
  endif()
target_link_libraries(mcdm_online_exploration lib  -lboost_unit_test_framework ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES} yaml-cpp)# ${OpenMP_CXX_LIBRARIES})
install(TARGETS mcdm_online_exploration RUNTIME DESTINATION bin)

# add_executable(random_walk random_walk.cpp  ${HEADER_FILES})
# # include the OpenMP compile flags as well
# # target_compile_options(random_walk PRIVATE)# ${OpenMP_CXX_FLAGS})
# # add the OpenMP libraries for linking
# target_link_libraries(random_walk lib  -lboost_unit_test_framework ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES} yaml-cpp)# ${OpenMP_CXX_LIBRARIES})
# install(TARGETS random_walk RUNTIME DESTINATION bin)

# add_executable(random_frontier random_frontier.cpp  ${HEADER_FILES})
# # include the OpenMP compile flags as well
# # target_compile_options(random_frontier PRIVATE)# ${OpenMP_CXX_FLAGS})
# # add the OpenMP libraries for linking
# target_link_libraries(random_frontier lib  -lboost_unit_test_framework ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES} yaml-cpp)# ${OpenMP_CXX_LIBRARIES})
# install(TARGETS random_frontier RUNTIME DESTINATION bin)


#add_executable(play_with_eigen play_with_eigen.cpp  ${HEADER_FILES})

#target_link_libraries(play_with_eigen lib  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

#install(TARGETS play_with_eigen RUNTIME DESTINATION bin)
#add_subdirectory(test)


#c++11
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -g  -ftest-coverage -fprofile-arcs")# -fopenmp -pg")

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
