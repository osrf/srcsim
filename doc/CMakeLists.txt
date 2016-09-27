find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/srcsim.in
                 ${CMAKE_BINARY_DIR}/srcsim.dox @ONLY)

  add_custom_target(doc
    # Generate the API documentation
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/srcsim.dox
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()
