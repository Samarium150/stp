include(FetchContent)
FetchContent_Declare(GoogleTest
  GIT_REPOSITORY "https://github.com/google/googletest"
  GIT_TAG main
  GIT_SHALLOW ON
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(GoogleTest)
enable_testing()

include(GoogleTest)
