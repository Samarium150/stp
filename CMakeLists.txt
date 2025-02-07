cmake_minimum_required(VERSION 3.24)
project(STP LANGUAGES CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "10")
    message(FATAL_ERROR "GCC version must be at least 10!")
    return()
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Wno-deprecated-declarations")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19")
    message(FATAL_ERROR "Clang version must be at least 19!")
    return()
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-deprecated-declarations -fexperimental-library")
elseif (MSVC)
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19.28")
    message(FATAL_ERROR "MSVC version must be at least 19.28!")
    return()
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /WX /wd4244")
endif()

set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

include(FetchContent)
FetchContent_Declare(ConcurrentQueue
  GIT_REPOSITORY "https://github.com/cameron314/concurrentqueue"
  GIT_TAG master
  GIT_SHALLOW ON
)
FetchContent_MakeAvailable(ConcurrentQueue)

file(GLOB HEADERS ${CMAKE_SOURCE_DIR}/include/*.h)

add_subdirectory(src)
add_subdirectory(test)
