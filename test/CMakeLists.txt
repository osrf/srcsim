include_directories (
  ${PROJECT_SOURCE_DIR}/test/gtest/include
  ${PROJECT_SOURCE_DIR}/test/gtest
  ${PROJECT_SOURCE_DIR}/test
)

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/test_config.h)

# Build gtest
add_library(gtest_${PROJECT_NAME} STATIC gtest/src/gtest-all.cc)
add_library(gtest_main_${PROJECT_NAME} STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main_${PROJECT_NAME} gtest_${PROJECT_NAME})
set(GTEST_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest_${PROJECT_NAME}.a")
set(GTEST_MAIN_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest_main_${PROJECT_NAME}.a")

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(regression)
