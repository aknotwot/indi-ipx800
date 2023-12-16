cmake_minimum_required(VERSION 3.0)
PROJECT(indi_ipx800_v4 C CXX)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
##LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")

include(GNUInstallDirs)

set(IPX800_V4_VERSION_MAJOR 0)
set(IPX800_V4_VERSION_MINOR 3)

find_package(INDI REQUIRED)
##find_package(ZLIB REQUIRED)
##find_package(Nova REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_ipx800_v4.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_nexdome.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
#include_directories( ${NOVA_INCLUDE_DIR})

include(CMakeCommon)

########### IPX800v4  ###########
set(indi_ipx800_v4_SRCS
   ${CMAKE_CURRENT_SOURCE_DIR}/indi_ipx800_v4.cpp
   )

add_executable(indi_ipx800_v4 ${indi_ipx800_v4_SRCS})

target_link_libraries(indi_ipx800_v4 ${INDI_LIBRARIES})

install(TARGETS indi_ipx800_v4 RUNTIME DESTINATION bin )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_ipx800_v4.xml DESTINATION ${INDI_DATA_DIR})
