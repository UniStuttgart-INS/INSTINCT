macro(run_CMakeRC)
  # * https://caiorss.github.io/C-Cpp-Notes/resources-executable.html#org7635e6a
  # * https://github.com/vector-of-bool/cmrc

  # Download automatically, you can also just copy the conan.cmake file
  if(NOT EXISTS "${CMAKE_BINARY_DIR}/CMakeRC.cmake")
    message(STATUS "Downloading CMakeRC.cmake from https://github.com/vector-of-bool/cmrc")
    file(DOWNLOAD "https://raw.githubusercontent.com/vector-of-bool/cmrc/master/CMakeRC.cmake"
         "${CMAKE_BINARY_DIR}/CMakeRC.cmake" STATUS status)

    list(GET status 0 error_code)

    if(error_code)
      message(STATUS "Could not download CMakeRC Cmake file, using local backup")
      file(COPY "cmake/CMakeRC-2584030/CMakeRC.cmake" DESTINATION "${CMAKE_BINARY_DIR}")
    endif()
  endif()

  message(STATUS "CMakeRC: Adding Resource library")

  include(${CMAKE_BINARY_DIR}/CMakeRC.cmake)

  cmrc_add_resource_library(
    instinct-resources
    ALIAS
    instinct::rc
    NAMESPACE
    instinct
    resources/images/BlueprintBackground.png
    resources/images/INSTINCT_Logo_Text_white_small.png
    resources/images/INSTINCT_Logo_Text_black_small.png
    resources/images/INS_logo_rectangular_white_small.png
    resources/images/INS_logo_rectangular_black_small.png)

  set_target_properties(instinct-resources PROPERTIES CXX_CLANG_TIDY "")
  set_target_properties(instinct-resources PROPERTIES CXX_CPPCHECK "")

  include_directories(SYSTEM ${CMRC_INCLUDE_DIR})
endmacro()
