include(FetchContent)
FetchContent_Declare(GoogleTest
  GIT_REPOSITORY "https://github.com/google/googletest"
  GIT_TAG main
  GIT_SHALLOW ON
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(GoogleTest)
enable_testing()

file(GLOB UTILS ${CMAKE_CURRENT_SOURCE_DIR}/test_utils.*)

file(GLOB UTILS ${CMAKE_CURRENT_SOURCE_DIR}/test_utils.*)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(gtest PRIVATE "-w")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(gtest PRIVATE "/w")
endif()

add_executable(test_standard_puzzle test_standard_puzzle.cpp ${UTILS} ${HEADERS})
target_include_directories(test_standard_puzzle PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(test_standard_puzzle PRIVATE GTest::gtest_main)

add_executable(test_modified_puzzle test_modified_puzzle.cpp ${UTILS} ${HEADERS})
target_include_directories(test_modified_puzzle PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(test_modified_puzzle PRIVATE GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_standard_puzzle)
gtest_discover_tests(test_modified_puzzle)
