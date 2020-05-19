cmake_minimum_required(VERSION 3.16)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(cmake/utils.cmake)
extract_project_version()

# Set the project name, version and language
project(
  "NavSoS"
  VERSION ${PROJECT_VERSION}
  LANGUAGES CXX)
string(TOLOWER ${CMAKE_PROJECT_NAME} PROJECT_NAME_LOWERCASE)
message(STATUS "${CMAKE_PROJECT_NAME}: ${PROJECT_VERSION}")

# ######################################################################################################################

# Basic settings
include(cmake/StandardProjectSettings.cmake)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_20)

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

# sanitizer options if supported by compiler
include(cmake/Sanitizers.cmake)
enable_sanitizers(project_options)

# enable doxygen
include(cmake/Doxygen.cmake)
enable_doxygen()

# allow for static analysis options
include(cmake/StaticAnalyzers.cmake)

# If true, the library target will be a shared library, otherwise it will be static.
option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)
option(ENABLE_TESTING "Enable Test Builds" OFF)
option(ENABLE_MAIN "Build Main file" ON)
option(ENABLE_GUI "Build GUI" OFF)

# Include all project headers
include_directories(src)

# ######################################################################################################################
# Conan Packages
# ######################################################################################################################

set(CONAN_EXTRA_REQUIRES "")
set(CONAN_EXTRA_OPTIONS "")

list(
  APPEND
  CONAN_EXTRA_REQUIRES
  fmt/6.1.2
  spdlog/1.5.0
  boost/1.72.0
  eigen/3.3.7
  catch2/2.12.1)

if(BUILD_SHARED_LIBS)
  list(APPEND CONAN_EXTRA_OPTIONS "Pkg:shared=True")
endif()

include(cmake/Conan.cmake)
run_conan()

# ######################################################################################################################
# Libraries (inlcude as SYSTEM, to prevent Static Analazers to scan them)
# ######################################################################################################################

option(ENABLE_VN_UB_SENSORS "Enable VectorNav and Ublox Sensors" ON)

if(ENABLE_VN_UB_SENSORS)
  # Include VectorNav library
  add_subdirectory(lib/vnproglib-1.1.5.0 libvncxx)
  include_directories(SYSTEM lib/vnproglib-1.1.5.0/include)
  set_target_properties(libvncxx PROPERTIES CXX_CLANG_TIDY "")
  set_target_properties(libvncxx PROPERTIES CXX_CPPCHECK "")

  # Include ublox library
  add_subdirectory(lib/ubloxlib libubcxx)
  include_directories(SYSTEM lib/ubloxlib/include)
  set_target_properties(libubcxx PROPERTIES CXX_CLANG_TIDY "")
  set_target_properties(libubcxx PROPERTIES CXX_CPPCHECK "")
endif()

# Include gnuplot iostream
include_directories(SYSTEM lib/gnuplot-iostream)

# ######################################################################################################################

# Find installed packages
find_package(Threads REQUIRED)

# ######################################################################################################################

# Specify output path for executable files
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin/Release)

if(ENABLE_GUI)
  # Include nodeeditor library
  add_subdirectory(lib/nodeeditor)
  set_target_properties(nodes PROPERTIES CXX_CLANG_TIDY "")
endif()

if(ENABLE_MAIN)
  add_subdirectory(src)
endif()

if(ENABLE_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

option(ENABLE_UNITY "Enable Unity builds of projects" OFF)
if(ENABLE_UNITY)
  # Add for any project you want to apply unity builds for
  set_target_properties(${PROJECT_NAME_LOWERCASE} PROPERTIES UNITY_BUILD ON)
  if(ENABLE_GUI)
    set_target_properties("${PROJECT_NAME_LOWERCASE}-gui" PROPERTIES UNITY_BUILD ON)
  endif()
endif()

# Copy compile_commands.json to root directory
add_custom_target(copy-compile-commands ALL ${CMAKE_COMMAND} -E copy_if_different
                                            ${CMAKE_BINARY_DIR}/compile_commands.json ${CMAKE_CURRENT_LIST_DIR})
