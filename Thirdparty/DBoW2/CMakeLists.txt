cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2_DBoW2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 ")

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(${PROJECT_NAME} SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(FILES ${HDRS_DBOW2} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ORB_SLAM2/DBoW2
  )
install(FILES ${HDRS_DUTILS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ORB_SLAM2/DUtils
  )
