project (CMT)

cmake_minimum_required (VERSION 2.6)

option(BUILD_TRAX_CLIENT "Build the trax client." OFF)
option(BUILD_SHARED_LIBS "Build shared libraries." ON)
find_package(OpenCV REQUIRED)
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Found OpenMP")
ENDIF()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_BUILD_TYPE Debug)
add_definitions ("-Wall")
add_definitions (-std=c++11)
include_directories(${OpenCV_INCLUDE_DIRS})

if(WIN32)
add_executable (cmt common.cpp gui.cpp main.cpp CMT_MAP.cpp
    CMT.cpp Config.cpp Consensus.cpp Fusion.cpp Matcher.cpp Tracker.cpp
    fastcluster/fastcluster.cpp getopt/getopt.cpp
    )
else()
add_executable (cmt common.cpp gui.cpp main.cpp CMT_MAP.cpp
    CMT.cpp Config.cpp Consensus.cpp Fusion.cpp Matcher.cpp Tracker.cpp
    fastcluster/fastcluster.cpp)
endif()

target_link_libraries(cmt ${OpenCV_LIBS})

add_library(cmtlib common.cpp CMT.cpp CMT_MAP.cpp Consensus.cpp Fusion.cpp Matcher.cpp
    Tracker.cpp fastcluster/fastcluster.cpp)
target_link_libraries(cmtlib ${OpenCV_LIBS})
set_target_properties(cmtlib PROPERTIES OUTPUT_NAME "cmt")


if(BUILD_TRAX_CLIENT)
    set(TRAX_DIR "" CACHE FILEPATH "Path to trax")

    include_directories(${TRAX_DIR}/include)

    add_executable (trax_client cmt common.cpp gui.cpp trax.cpp
        CMT.cpp Consensus.cpp Fusion.cpp Matcher.cpp Tracker.cpp
        fastcluster/fastcluster.cpp)

    find_library(TRAX_LIBRARY NAMES trax HINTS "${TRAX_DIR}/lib")

    target_link_libraries(trax_client ${OpenCV_LIBS} ${TRAX_LIBRARY})

endif()

