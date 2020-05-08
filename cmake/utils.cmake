# Get version from src/util/version.hpp and put it in PROJECT_VERSION
function(extract_project_version)
  file(READ "${CMAKE_CURRENT_LIST_DIR}/src/util/Version.hpp" file_contents)

  string(REGEX MATCH "PROJECT_VER_MAJOR ([0-9]+)" _ "${file_contents}")
  if(NOT CMAKE_MATCH_COUNT EQUAL 1)
    message(FATAL_ERROR "Could not extract major version number from util/Version.hpp")
  endif()
  set(ver_major ${CMAKE_MATCH_1})

  string(REGEX MATCH "PROJECT_VER_MINOR ([0-9]+)" _ "${file_contents}")
  if(NOT CMAKE_MATCH_COUNT EQUAL 1)
    message(FATAL_ERROR "Could not extract minor version number from util/Version.hpp")
  endif()
  set(ver_minor ${CMAKE_MATCH_1})

  string(REGEX MATCH "PROJECT_VER_PATCH ([0-9]+)" _ "${file_contents}")
  if(NOT CMAKE_MATCH_COUNT EQUAL 1)
    message(FATAL_ERROR "Could not extract patch version number from util/Version.hpp")
  endif()
  set(ver_patch ${CMAKE_MATCH_1})

  set(PROJECT_VERSION
      "${ver_major}.${ver_minor}.${ver_patch}"
      PARENT_SCOPE)
endfunction()
