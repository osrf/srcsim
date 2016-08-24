include (${project_cmake_dir}/Utils.cmake)

set (headers
)

set (srcsim_headers "" CACHE INTERNAL "SRC Sim headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(ign_headers
    "SRC Sim headers" "#include <srcsim/${hdr}>\n")
endforeach()

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/srcsim.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/srcsim.hh)

srcsim_install_includes("${INCLUDE_INSTALL_DIR_POSTFIX}/${IGN_PROJECT_NAME}"
  ${CMAKE_CURRENT_BINARY_DIR}/srcsim.hh)
srcsim_install_includes("${INCLUDE_INSTALL_DIR_POSTFIX}/${IGN_PROJECT_NAME}"
  ${headers})
