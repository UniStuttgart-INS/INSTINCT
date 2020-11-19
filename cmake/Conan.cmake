macro(run_conan)
  # Download automatically, you can also just copy the conan.cmake file
  if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://github.com/conan-io/cmake-conan/raw/v0.15/conan.cmake" "${CMAKE_BINARY_DIR}/conan.cmake"
         STATUS status)

    list(GET status 0 error_code)

    if(error_code)
      message(STATUS "Could not download Conan Cmake file, using local backup")
      file(COPY "cmake/Conan-v0.15/conan.cmake" DESTINATION "${CMAKE_BINARY_DIR}")
    endif()
  endif()

  include(${CMAKE_BINARY_DIR}/conan.cmake)

  conan_add_remote(NAME bincrafters URL https://api.bintray.com/conan/bincrafters/public-conan)

  # See https://github.com/conan-io/cmake-conan/blob/release/0.15/README.md for settings
  conan_cmake_run(
    REQUIRES
    ${CONAN_EXTRA_REQUIRES}
    CONANFILE
    conanfile.txt
    OPTIONS
    ${CONAN_EXTRA_OPTIONS}
    BASIC_SETUP
    CMAKE_TARGETS # individual targets to link to
    BUILD
    missing)
endmacro()
