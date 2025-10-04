if (NOT MINIBOT_DRIVER_PATH)
    set(MINIBOT_DRIVER_PATH ${CMAKE_CURRENT_LIST_DIR})
endif ()

set(MINIBOT_DRIVER_PATH ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "Path to the Minibot driver" FORCE)
# list(APPEND CMAKE_MODULE_PATH ${MINIBOT_DRIVER_PATH})
message("MINIBOT_DRIVER_PATH is ${CMAKE_CURRENT_LIST_DIR}")
if(DEFINED MINIBOT_DRIVER_BIN_PATH)
    message("MINIBOT_DRIVER_BIN_PATH is ${MINIBOT_DRIVER_BIN_PATH}")
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MINIBOT_DRIVER_BIN_PATH})
endif()

set(MINIBOT_DRIVER_BUILD_TEST   FALSE  )
message("  MINIBOT_DRIVER_BUILD_TEST   is ${MINIBOT_DRIVER_BUILD_TEST}")

add_subdirectory(${MINIBOT_DRIVER_PATH} minibot)
unset (CMAKE_RUNTIME_OUTPUT_DIRECTORY)

# ## ##################################################################
# The following template allows you to remotely import minibot-driver
# into your own project using CMake.
# Just save the code between >>><<< as minibot-driver.cmake and include()
# the file in your top-most CMakeLists.txt file as follows:
#
# include(minibot-driver.cmake)
#
# ## ##################################################################
# BEGIN minibot-driver.cmake >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# # This file can be placed into an external project to help locate the minibot driver
# # It should be include()ed prior to project()
#
# if (DEFINED ENV{MINIBOT_DRIVER_PATH} AND (NOT MINIBOT_DRIVER_PATH))
#     set(MINIBOT_DRIVER_PATH $ENV{MINIBOT_DRIVER_PATH})
#     message("Using MINIBOT_DRIVER_PATH from environment ('${MINIBOT_DRIVER_PATH}')")
# endif ()
#
# if (DEFINED ENV{MINIBOT_DRIVER_FETCH_FROM_GIT} AND (NOT MINIBOT_DRIVER_FETCH_FROM_GIT))
#     set(MINIBOT_DRIVER_FETCH_FROM_GIT $ENV{MINIBOT_DRIVER_FETCH_FROM_GIT})
#     message("Using MINIBOT_DRIVER_FETCH_FROM_GIT from environment ('${MINIBOT_DRIVER_FETCH_FROM_GIT}')")
# endif ()
#
# if (DEFINED ENV{MINIBOT_DRIVER_FETCH_FROM_GIT_PATH} AND (NOT MINIBOT_DRIVER_FETCH_FROM_GIT_PATH))
#     set(MINIBOT_DRIVER_FETCH_FROM_GIT_PATH $ENV{MINIBOT_DRIVER_FETCH_FROM_GIT_PATH})
#     message("Using MINIBOT_DRIVER_FETCH_FROM_GIT_PATH from environment ('${MINIBOT_DRIVER_FETCH_FROM_GIT_PATH}')")
# endif ()
#
# set(MINIBOT_DRIVER_PATH "${MINIBOT_DRIVER_PATH}" CACHE PATH "Path to the tcpMinibot API")
# set(MINIBOT_DRIVER_FETCH_FROM_GIT "${MINIBOT_DRIVER_FETCH_FROM_GIT}" CACHE BOOL "Set to ON to download the tcpMinibot API from git if unlocatable")
# set(MINIBOT_DRIVER_FETCH_FROM_GIT_PATH "${MINIBOT_DRIVER_FETCH_FROM_GIT_PATH}" CACHE FILEPATH "location to download tcpMinibot API")
#
# if (NOT MINIBOT_DRIVER_PATH)
#     if (MINIBOT_DRIVER_FETCH_FROM_GIT)
#         include(FetchContent)
#         set(FETCHCONTENT_BASE_DIR_SAVE ${FETCHCONTENT_BASE_DIR})
#         if (MINIBOT_DRIVER_FETCH_FROM_GIT_PATH)
#             get_filename_component(FETCHCONTENT_BASE_DIR "${MINIBOT_DRIVER_FETCH_FROM_GIT_PATH}" REALPATH BASE_DIR "${CMAKE_SOURCE_DIR}")
#         endif ()
#
#         FetchContent_Declare(
#             minibotdriver
#             GIT_REPOSITORY https://github.com/kyordhel/minibot-driver
#             GIT_TAG master
#         )
#
#         if (NOT tcpMinibot)
#             message("Downloading the Minibot driver")
#             FetchContent_Populate(minibotdriver)
#             set(MINIBOT_DRIVER_PATH ${minibottcpapi_SOURCE_DIR})
#         endif ()
#         set(FETCHCONTENT_BASE_DIR ${FETCHCONTENT_BASE_DIR_SAVE})
#     else ()
#         message(FATAL_ERROR
#                 "API location was not specified. Please set MINIBOT_DRIVER_PATH or set MINIBOT_DRIVER_FETCH_FROM_GIT to on to fetch from git."
#                 )
#     endif ()
# endif ()
#
# get_filename_component(MINIBOT_DRIVER_PATH "${MINIBOT_DRIVER_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
# if (NOT EXISTS ${MINIBOT_DRIVER_PATH})
#     message(FATAL_ERROR "Directory '${MINIBOT_DRIVER_PATH}' not found")
# endif ()
#
# message("MINIBOT_DRIVER_PATH: ${MINIBOT_DRIVER_PATH}")
#
# set(MINIBOT_DRIVER_IMPORT_CMAKE_FILE ${MINIBOT_DRIVER_PATH}/minibot-driver_import.cmake)
# if (NOT EXISTS ${MINIBOT_DRIVER_IMPORT_CMAKE_FILE})
#     message(FATAL_ERROR "Directory '${MINIBOT_DRIVER_PATH}' does not appear to contain the Minibot driver")
# endif ()
#
# set(MINIBOT_DRIVER_PATH ${MINIBOT_DRIVER_PATH} CACHE PATH "Path to the Minibot driver" FORCE)
#
# include(${MINIBOT_DRIVER_IMPORT_CMAKE_FILE})
#
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< END minibot-driver.cmake
