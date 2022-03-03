# Options
option(ENABLE_DOXYGEN "Enable doxygen doc builds of source" FALSE)
option(DOC_INTERNAL "Build internal documentation" OFF)
# option(DOC_USE_MATHJAX "Use MathJax for rendering math in HTML docs" ON)
option(DOC_CHECK_CODE_DOCUMENTATION "Check if all code is documented" OFF)

if(ENABLE_DOXYGEN)
  # Search for package
  find_package(Doxygen REQUIRED dot OPTIONAL_COMPONENTS mscgen dia)

  if(DOXYGEN_FOUND)
    # Set some Doxygen flags
    set(INSTINCT_DOXY_PROJECT_NAME             "INSTINCT")
    set(INSTINCT_DOXY_OUTPUT_DIRECTORY_SUFFIX  "")
    set(INSTINCT_DOXY_INPUT                    "\"${INSTINCT_SOURCE_DIR}/src\" \"${INSTINCT_SOURCE_DIR}/doc\"")
    set(INSTINCT_DOXY_HTML_COLORSTYLE_HUE      "220")

    if(DOC_CHECK_CODE_DOCUMENTATION)
      set(INSTINCT_DOXY_EXCLUDE                "")
    #   set(INSTINCT_DOXY_ALIASES                "eqref{1}=")
      set(INSTINCT_DOXY_WARN_AS_ERROR          "FAIL_ON_WARNINGS")
      set(INSTINCT_DOXY_WARN_IF_UNDOCUMENTED   "YES")
      set(INSTINCT_DOXY_WARN_IF_INCOMPLETE_DOC "YES")
      set(INSTINCT_DOXY_QUIET                  "YES")
      set(INSTINCT_DOXY_EXTRACT_PRIVATE        "YES")
      set(INSTINCT_DOXY_EXTRACT_PRIV_VIRTUAL   "YES")
      set(INSTINCT_DOXY_EXTRACT_LOCAL_CLASSES  "YES")
      set(INSTINCT_DOXY_EXTRACT_LOCAL_METHODS  "YES")
      set(INSTINCT_DOXY_HIDE_UNDOC_MEMBERS     "NO")
      set(INSTINCT_DOXY_HIDE_UNDOC_CLASSES     "NO")
      set(INSTINCT_DOXY_HIDE_FRIEND_COMPOUNDS  "NO")
    else()
      set(INSTINCT_DOXY_EXCLUDE                ${CMAKE_SOURCE_DIR}/src/internal)
    #   set(INSTINCT_DOXY_ALIASES                "")
      set(INSTINCT_DOXY_WARN_AS_ERROR          "NO")
      set(INSTINCT_DOXY_WARN_IF_UNDOCUMENTED   "NO")
      set(INSTINCT_DOXY_WARN_IF_INCOMPLETE_DOC "NO")
      set(INSTINCT_DOXY_QUIET                  "NO")
      set(INSTINCT_DOXY_EXTRACT_PRIVATE        "NO")
      set(INSTINCT_DOXY_EXTRACT_PRIV_VIRTUAL   "NO")
      set(INSTINCT_DOXY_EXTRACT_LOCAL_CLASSES  "NO")
      set(INSTINCT_DOXY_EXTRACT_LOCAL_METHODS  "NO")
      set(INSTINCT_DOXY_HIDE_UNDOC_MEMBERS     "YES")
      set(INSTINCT_DOXY_HIDE_UNDOC_CLASSES     "YES")
      set(INSTINCT_DOXY_HIDE_FRIEND_COMPOUNDS  "YES")
    endif()

    if(DOC_INTERNAL)
      set(INSTINCT_DOXY_INTERNAL               "YES")
    else()
      set(INSTINCT_DOXY_INTERNAL               "NO")
    endif()

    # if (DOC_USE_MATHJAX)
    #   set(INSTINCT_DOXY_USE_MATHJAX            "YES")
    # else ()
    #   set(INSTINCT_DOXY_USE_MATHJAX            "NO")
    # endif()

    configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
      ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    )
    configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/DoxygenLayout.xml
      ${CMAKE_CURRENT_BINARY_DIR}/DoxygenLayout.xml
    )

    configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/header.html
      ${CMAKE_CURRENT_BINARY_DIR}/header.html
    )

    configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/footer.html
      ${CMAKE_CURRENT_BINARY_DIR}/footer.html
    )
    configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/customdoxygen.css
      ${CMAKE_CURRENT_BINARY_DIR}/customdoxygen.css
    )

    add_custom_target(
        doc-prerequisites
        ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory ${INSTINCT_BINARY_DIR}/doc/html/ ${INSTINCT_BINARY_DIR}/doc/html/resources
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/eigen_navtree_hacks.js           ${INSTINCT_BINARY_DIR}/doc/html${INSTINCT_DOXY_OUTPUT_DIRECTORY_SUFFIX}/
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/ftv2pnode.png                    ${INSTINCT_BINARY_DIR}/doc/html${INSTINCT_DOXY_OUTPUT_DIRECTORY_SUFFIX}/
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/resources/*                      ${INSTINCT_BINARY_DIR}/doc/html${INSTINCT_DOXY_OUTPUT_DIRECTORY_SUFFIX}/resources/
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      )

    add_custom_target(doc ALL
      COMMAND doxygen
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

    add_dependencies(doc doc-prerequisites)

  else()
    message(SEND_ERROR "Doxygen need to be installed to generate the doxygen documentation")
  endif()
endif()