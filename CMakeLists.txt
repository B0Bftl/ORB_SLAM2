cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 ")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(G2O REQUIRED)
if(${G2O_CORE_LIBRARY_DEBUG} AND  ${G2O_TYPES_SBA_DEBUG} AND {G2O_TYPES_SIM3_DEBUG} AND {G2O_SOLVER_EIGEN_DEBUG})
set(G2O_LIBS
  optimized ${G2O_CORE_LIBRARY} debug ${G2O_CORE_LIBRARY_DEBUG}
  optimized ${G2O_TYPES_SBA} debug ${G2O_TYPES_SBA_DEBUG}
  optimized ${G2O_TYPES_SIM3} debug ${G2O_TYPES_SIM3_DEBUG}
  optimized ${G2O_SOLVER_EIGEN} debug ${G2O_SOLVER_EIGEN_DEBUG}
  )
else(${G2O_CORE_LIBRARY_DEBUG} AND  ${G2O_TYPES_SBA_DEBUG} AND {G2O_TYPES_SIM3_DEBUG} AND {G2O_SOLVER_EIGEN_DEBUG})
  set(G2O_LIBS  ${G2O_CORE_LIBRARY} ${G2O_TYPES_SBA}  ${G2O_TYPES_SIM3} ${G2O_SOLVER_EIGEN})
endif(${G2O_CORE_LIBRARY_DEBUG} AND  ${G2O_TYPES_SBA_DEBUG} AND {G2O_TYPES_SIM3_DEBUG} AND {G2O_SOLVER_EIGEN_DEBUG})

find_library(DBoW2_LIB NAMES ORB_SLAM2_DBoW2 PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib)
find_path(DBoW2_INCLUDE_DIR NAMES DBoW2/BowVector.h
  PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2
  PATH_SUFFIXES ORB_SLAM2)
if(NOT DBoW2_LIB OR NOT DBoW2_INCLUDE_DIR)
  message(FATAL_ERROR "DBoW2 library not found.")
endif(NOT DBoW2_LIB OR NOT DBoW2_INCLUDE_DIR)

include_directories(
${PROJECT_SOURCE_DIR}/include
${DBoW2_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
${G2O_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)
  
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${G2O_LIBS}
${Pangolin_LIBRARIES}
${DBoW2_LIB}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  )
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ORB_SLAM2
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.h"
  )

# Build tools
set(BUILD_TOOLS ON CACHE BOOL "Enable building tools")
if(BUILD_TOOLS)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
  add_executable(bin_vocabulary
    tools/bin_vocabulary.cc)
  target_link_libraries(bin_vocabulary ${PROJECT_NAME})
endif(BUILD_TOOLS)

# Build examples
set(BUILD_EXAMPLES ON CACHE BOOL "Enable building examples")
set(BUILD_EXAMPLES ON)
if(BUILD_EXAMPLES)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

  add_executable(rgbd_tum
    Examples/RGB-D/rgbd_tum.cc)
  target_link_libraries(rgbd_tum ${PROJECT_NAME})

  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

  add_executable(stereo_kitti
    Examples/Stereo/stereo_kitti.cc)
  target_link_libraries(stereo_kitti ${PROJECT_NAME})

  add_executable(stereo_euroc
    Examples/Stereo/stereo_euroc.cc)
  target_link_libraries(stereo_euroc ${PROJECT_NAME})


  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

  add_executable(mono_tum
    Examples/Monocular/mono_tum.cc)
  target_link_libraries(mono_tum ${PROJECT_NAME})

  add_executable(mono_kitti
    Examples/Monocular/mono_kitti.cc)
  target_link_libraries(mono_kitti ${PROJECT_NAME})

  add_executable(mono_euroc
    Examples/Monocular/mono_euroc.cc)
  target_link_libraries(mono_euroc ${PROJECT_NAME})
endif(BUILD_EXAMPLES)
