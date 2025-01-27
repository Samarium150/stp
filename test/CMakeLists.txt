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

add_executable(test test.cpp ${HEADERS})
target_include_directories(test PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(test GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test)
