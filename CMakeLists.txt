cmake_minimum_required(VERSION 3.16)

include(cmake/utils.cmake)
extract_project_version()

# Set the project name, version and language
project("NavSoS" VERSION ${PROJECT_VERSION} LANGUAGES CXX)
string(TOLOWER ${CMAKE_PROJECT_NAME} PROJECT_NAME_LOWERCASE)
message(STATUS "${CMAKE_PROJECT_NAME}: ${PROJECT_VERSION}")

# Basic settings
include(cmake/StandardProjectSettings.cmake)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

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

# If true, the library target will be a shared library, otherwise it will be static.
option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)
option(ENABLE_TESTING "Enable Test Builds" OFF)
option(ENABLE_GUI "Build GUI" OFF)

# Include all project headers
include_directories(src)

# Conan Packages
set(CONAN_EXTRA_REQUIRES "")
set(CONAN_EXTRA_OPTIONS "")

list(APPEND CONAN_EXTRA_REQUIRES fmt/6.1.2
                                 spdlog/1.5.0
                                 boost/1.72.0
                                 eigen/3.3.7
                                 gtest/1.10.0)

if (BUILD_SHARED_LIBS)
    list(APPEND CONAN_EXTRA_OPTIONS "Pkg:shared=True")
endif()

include(cmake/Conan.cmake)
run_conan()

# Include VectorNav library
add_subdirectory(lib/vnproglib-1.1.5.0 libvncxx)
include_directories(lib/vnproglib-1.1.5.0/include)

# Include ublox library
add_subdirectory(lib/ubloxlib libubcxx)
include_directories(lib/ubloxlib/include)

# Include gnuplot iostream
include_directories(lib/gnuplot-iostream)

# Find installed packages
find_package (Threads REQUIRED)

# Specify output path for executable files
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin/Release)

if(ENABLE_TESTING)
    # enable_testing()
    add_subdirectory(tests)
endif()

add_subdirectory(src)

if(ENABLE_GUI)
    # Include nodeeditor library
    add_subdirectory(lib/nodeeditor)

    add_subdirectory(src/gui)
endif()