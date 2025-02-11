FetchContent_Declare(GoogleTest
  GIT_REPOSITORY "https://github.com/google/googletest"
  GIT_TAG main
  GIT_SHALLOW ON
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(GoogleTest)
enable_testing()

file(GLOB UTILS ${CMAKE_CURRENT_SOURCE_DIR}/test_utils.*)

file(GLOB TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)

file(GLOB UTILS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test_utils.*)

list(FILTER TESTS EXCLUDE REGEX "utils")

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(gtest PRIVATE "-w")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(gtest PRIVATE "/w")
endif()

include(GoogleTest)
foreach (TEST_SOURCE_FILE ${TESTS})
    get_filename_component(name ${TEST_SOURCE_FILE} NAME_WE)
    add_executable(${name} ${TEST_SOURCE_FILE} ${UTILS} ${HEADERS})
    target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/include)
    target_link_libraries(${name} PRIVATE GTest::gtest_main concurrentqueue)
    gtest_discover_tests(${name})
endforeach()
