cmake_minimum_required(VERSION 3.14)
project(minibot)

## Compile as gnu11
set(CMAKE_C_STANDARD 11)
## Compile as C++14
# set(CMAKE_CXX_STANDARD 14)
# find_package(Boost REQUIRED COMPONENTS thread filesystem)

if( NOT DEFINED MINIBOT_DRIVER_BUILD_TEST )
  set(MINIBOT_DRIVER_BUILD_TEST TRUE)
endif()

set(ROOT_BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
set(ROOT_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)

set(EXECUTABLE_OUTPUT_PATH "${ROOT_BIN_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT_LIB_DIR}")

# ## ##################################################################
# Build library
# ## ##################################################################
file(GLOB MINIBOT_SRC
  ${PROJECT_SOURCE_DIR}/src/*.c
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(minibot-driver STATIC
  ${MINIBOT_SRC}
)

target_include_directories(minibot-driver
  PRIVATE
  ${PROJECT_SOURCE_DIR}/include/minibot
)

target_include_directories(minibot-driver
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)


# ## ##################################################################
# Build testing binaries
# ## ##################################################################
if( MINIBOT_DRIVER_BUILD_TEST )

add_executable(test-base
  ${PROJECT_SOURCE_DIR}/test/base.c
)

target_link_libraries(test-base
  minibot-driver
)

add_executable(test-lidar
  ${PROJECT_SOURCE_DIR}/test/lidar.c
)

target_link_libraries(test-lidar
  minibot-driver
)


add_executable(test-linesense
  ${PROJECT_SOURCE_DIR}/test/linesense.c
)

target_link_libraries(test-linesense
  minibot-driver
)


add_executable(test-dist
  ${PROJECT_SOURCE_DIR}/test/dist.c
)

target_link_libraries(test-dist
  minibot-driver
)


add_executable(test-turret
  ${PROJECT_SOURCE_DIR}/test/turret.c
)

target_link_libraries(test-turret
  minibot-driver
)


set(CURSES_NEED_NCURSES TRUE)
set(CURSES_NEED_WIDE TRUE)
find_package(Curses REQUIRED)

add_executable(test-minibot
  ${PROJECT_SOURCE_DIR}/test/minibot.c
)

target_link_libraries(test-minibot
  minibot-driver
  ${CURSES_LIBRARIES}
)

endif( MINIBOT_DRIVER_BUILD_TEST )

# target_link_libraries(Dijkstra
#   Boost::filesystem
#   Boost::thread
#   ${BOOST_LIBRARIES}
# )
