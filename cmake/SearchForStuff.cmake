include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

# Detect the architecture
include (${project_cmake_dir}/TargetArch.cmake)
target_architecture(ARCH)
message(STATUS "Building for arch: ${ARCH}")

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/ 
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()

#################################################
# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(EXTRA_CMAKE_CXX_FLAGS "${EXTRA_CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#################################################
# Find ignition transport
find_package(ignition-transport2 QUIET)
if (NOT ignition-transport2_FOUND)
  BUILD_WARNING ("Missing: Ignition Transport (libignition-transport2-dev)")
else()
  set (EXTRA_CMAKE_CXX_FLAGS "${EXTRA_CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
  include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
  link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})
endif()
