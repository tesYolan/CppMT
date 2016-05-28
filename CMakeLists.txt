project (CMT)

cmake_minimum_required (VERSION 2.6)

option(BUILD_SHARED_LIBS "Build shared libraries." ON)
option(BUILD_WRAPPER "Build C and Python wrapper." ON)
option(BUILD_TRAX_CLIENT "Build the trax client." OFF)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_definitions ("-Wall")


add_library(cmtlib common.cpp CMT.cpp CMT_MAP.cpp Consensus.cpp Fusion.cpp Matcher.cpp
    Tracker.cpp fastcluster/fastcluster.cpp)
target_link_libraries(cmtlib ${OpenCV_LIBS} ${Boost_LIBRARIES})
set_target_properties(cmtlib PROPERTIES OUTPUT_NAME "cmt")


add_executable(cmt gui.cpp main.cpp)
target_link_libraries(cmt cmtlib)

install (TARGETS cmtlib DESTINATION lib)
install (FILES CMT.h common.h Matcher.h Tracker.h cwrapper.h gui.h Fusion.h Consensus.h DESTINATION include/)
install (FILES logging/log.h DESTINATION include/logging)
install (FILES getopt/getopt.h DESTINATION include/getopt)
install (FILES fastcluster/fastcluster.h DESTINATION include/fastcluster)

if(BUILD_WRAPPER)
    add_library(cmtclib cwrapper.cpp)
    target_link_libraries(cmtclib cmtlib)
    set_target_properties(cmtclib PROPERTIES OUTPUT_NAME "ccmt")

    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/pycmt.py
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
endif()

if(BUILD_TRAX_CLIENT)
    set(TRAX_DIR "" CACHE FILEPATH "Path to trax")

    include_directories(${TRAX_DIR}/include)

    add_executable (trax_client trax.cpp)
    target_link_libraries(trax_client cmtlib)

    find_library(TRAX_LIBRARY NAMES trax HINTS "${TRAX_DIR}/lib")

    target_link_libraries(trax_client ${TRAX_LIBRARY})

endif()
