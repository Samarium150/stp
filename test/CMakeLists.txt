include(FetchContent)
FetchContent_Declare(GoogleTest
  GIT_REPOSITORY "https://github.com/google/googletest"
  GIT_TAG main
  GIT_SHALLOW ON
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(GoogleTest)
enable_testing()

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${CMAKE_BINARY_DIR}/test)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(gtest PRIVATE "-w")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(gtest PRIVATE "/w")
endif()

add_executable(test test.cpp ${HEADERS})
target_include_directories(test PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(test GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test)
