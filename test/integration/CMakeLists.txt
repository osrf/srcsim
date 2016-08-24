set(TEST_TYPE "INTEGRATION")

set(tests
)

link_directories(${PROJECT_BINARY_DIR}/test)

srcsim_build_tests(${tests})
