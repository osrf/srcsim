include (${project_cmake_dir}/Utils.cmake)

# set(CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
srcsim_add_executable(qual1 qual1.cc)
target_link_libraries(qual1 ${GAZEBO_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})

if (UNIX)
  target_link_libraries(qual1 pthread)
endif()

# srcsim_add_library(${PROJECT_LIBRARY_TARGET_NAME} ${sources})
# 
# # When the minimum CMake required version will be >= 3.1 
# # we could use the target_compile_features() command 
# # to enable C++11 support in a platform-independent way. 
# if(NOT MSVC)
#   target_compile_options(${PROJECT_LIBRARY_TARGET_NAME} PUBLIC "-std=c++11")
# endif()
# 
# target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
#   PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
#          $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR_FULL}>)
# 
#srcsim_build_tests(${gtest_sources})
#srcsim_install_library(${PROJECT_LIBRARY_TARGET_NAME} ${PROJECT_EXPORT_NAME})
