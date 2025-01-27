add_executable(stp main.cpp ${HEADERS})
target_include_directories(stp PRIVATE ${CMAKE_SOURCE_DIR}/include)
