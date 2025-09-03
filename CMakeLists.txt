cmake_minimum_required(VERSION 3.14)
project(minibot)

## Compile as C11
add_compile_options(-std=gnu11)
## Compile as C++14
# add_compile_options(-std=c++14)
# find_package(Boost REQUIRED COMPONENTS thread filesystem)

set(ROOT_BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
set(ROOT_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)

set(EXECUTABLE_OUTPUT_PATH "${ROOT_BIN_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT_LIB_DIR}")


file(GLOB MINIBOT_SRC
  ${PROJECT_SOURCE_DIR}/src/*.c
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(minibot STATIC
  ${MINIBOT_SRC}
)

target_include_directories(minibot
  PRIVATE
  ${PROJECT_SOURCE_DIR}/include/minibot
)

target_include_directories(minibot
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)


add_executable(test-base
  ${PROJECT_SOURCE_DIR}/test/base.c
)

target_link_libraries(test-base
  minibot
)

add_executable(test-lidar
  ${PROJECT_SOURCE_DIR}/test/lidar.c
)

target_link_libraries(test-lidar
  minibot
)

add_executable(test-linesense
  ${PROJECT_SOURCE_DIR}/test/linesense.c
)

target_link_libraries(test-linesense
  minibot
)

add_executable(test-turret
  ${PROJECT_SOURCE_DIR}/test/turret.c
)

target_link_libraries(test-turret
  minibot
)

set(CURSES_NEED_NCURSES TRUE)
set(CURSES_NEED_WIDE TRUE)
find_package(Curses REQUIRED)

add_executable(test-minibot
  ${PROJECT_SOURCE_DIR}/test/minibot.c
)

target_link_libraries(test-minibot
  minibot
  ${CURSES_LIBRARIES}
)

# target_link_libraries(Dijkstra
#   Boost::filesystem
#   Boost::thread
#   ${BOOST_LIBRARIES}
# )
