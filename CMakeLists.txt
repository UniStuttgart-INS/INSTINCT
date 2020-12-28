cmake_minimum_required(VERSION 3.15)

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

# Export the SDKROOT Environment Variable for MacOS
if(APPLE)
  if(NOT DEFINED ENV{SDKROOT})
    execute_process(COMMAND xcrun --show-sdk-path OUTPUT_VARIABLE sdkpath)
    string(REGEX REPLACE "\n$" "" sdkpath "${sdkpath}")
    set(ENV{SDKROOT} ${sdkpath})
    message(STATUS "SDK-Path: $ENV{SDKROOT}")
  endif()
endif()

# Basic settings
include(cmake/StandardProjectSettings.cmake)

# Prevent cmake builds with source dir .
include(cmake/PreventInSourceBuilds.cmake)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
  option(ENABLE_BUILD_WITH_TIME_TRACE "Enable -ftime-trace to generate time tracing .json files on clang" OFF)
  if(ENABLE_BUILD_WITH_TIME_TRACE)
    add_compile_definitions(project_options INTERFACE -ftime-trace)
  endif()
endif()

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# enable cache system
include(cmake/Cache.cmake)

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

# Logging Level
set(LOG_LEVEL
    "INFO"
    CACHE STRING "DATA|TRACE|DEBUG|INFO|WARN|ERROR|CRITICAL|OFF")

# Include all project headers
include_directories(src)

# ######################################################################################################################
# Conan Packages
# ######################################################################################################################

set(CONAN_EXTRA_REQUIRES "")
set(CONAN_EXTRA_OPTIONS "")

if(BUILD_SHARED_LIBS)
  list(APPEND CONAN_EXTRA_OPTIONS "Pkg:shared=True")
endif()

include(cmake/Conan.cmake)
run_conan()

# ######################################################################################################################
# Libraries (inlcude as SYSTEM, to prevent Static Analazers to scan them)
# ######################################################################################################################

# ------------------------------------------------- imgui ------------------------------------------------

file(
  GLOB _imgui_Sources
  LIST_DIRECTORIES OFF
  "${CMAKE_SOURCE_DIR}/lib/imgui/*.cpp" "${CMAKE_SOURCE_DIR}/lib/imgui/*.h")
add_library(imgui ${_imgui_Sources})
include_directories(SYSTEM "${CMAKE_SOURCE_DIR}/lib/imgui")
set_target_properties(imgui PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(imgui PROPERTIES CXX_CPPCHECK "")

# ------------------------------------------- imgui-node-editor ------------------------------------------ */

# Define IMGUI_NODE_EDITOR_ROOT_DIR pointing to project root directory
get_filename_component(IMGUI_NODE_EDITOR_ROOT_DIR ${CMAKE_SOURCE_DIR}/lib/imgui-node-editor ABSOLUTE CACHE)
# Point CMake where to look for module files.
list(APPEND CMAKE_MODULE_PATH ${IMGUI_NODE_EDITOR_ROOT_DIR}/misc/cmake-modules)
find_package(imgui REQUIRED)
find_package(imgui_node_editor REQUIRED)
target_link_libraries(imgui_node_editor PRIVATE imgui)
include_directories(SYSTEM lib/imgui-node-editor)
# include_directories(SYSTEM lib/imgui-node-editor/external/imgui)
set_target_properties(imgui_node_editor PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(imgui_node_editor PROPERTIES CXX_CPPCHECK "")

# -------------------------------------------- ImGuiFileDialog ------------------------------------------- */

add_library(
  ImGuiFileDialog
  "${CMAKE_SOURCE_DIR}/lib/ImGuiFileDialog/ImGuiFileDialog/ImGuiFileDialog.cpp"
  "${CMAKE_SOURCE_DIR}/lib/ImGuiFileDialog/ImGuiFileDialog/ImGuiFileDialog.h"
  "${CMAKE_SOURCE_DIR}/lib/ImGuiFileDialog/ImGuiFileDialog/ImGuiFileDialogConfig.h")
include_directories(SYSTEM lib/ImGuiFileDialog/ImGuiFileDialog)
set_target_properties(ImGuiFileDialog PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(ImGuiFileDialog PROPERTIES CXX_CPPCHECK "")

# ------------------------------------------------ implot ------------------------------------------------ */

file(
  GLOB _implot_Sources
  LIST_DIRECTORIES OFF
  "${CMAKE_SOURCE_DIR}/lib/implot/*.cpp" "${CMAKE_SOURCE_DIR}/lib/implot/*.h")
add_library(implot ${_implot_Sources})
include_directories(SYSTEM "${CMAKE_SOURCE_DIR}/lib/implot")
set_target_properties(implot PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(implot PROPERTIES CXX_CPPCHECK "")

# ---------------------------------------------- Application --------------------------------------------- */

add_subdirectory(lib/application)
include_directories(SYSTEM lib/application/include)
target_link_libraries(application PRIVATE implot)
set_target_properties(application PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(application PROPERTIES CXX_CPPCHECK "")

# ----------------------------------------------- VectorNav ---------------------------------------------- */

add_subdirectory(lib/vnproglib libvncxx)
include_directories(SYSTEM lib/vnproglib/include)
set_target_properties(libvncxx PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(libvncxx PROPERTIES CXX_CPPCHECK "")

# ------------------------------------------------ Navio2 ------------------------------------------------ */

if(NOT APPLE)
  # Include navio2 library
  add_subdirectory(lib/navio2 libnavio)
  include_directories(SYSTEM lib/navio2/include)
  set_target_properties(libnavio PROPERTIES CXX_CLANG_TIDY "")
  set_target_properties(libnavio PROPERTIES CXX_CPPCHECK "")
endif()

# ------------------------------------------------- UART ------------------------------------------------- */

add_subdirectory(lib/uartsensor libUartSensor)
include_directories(SYSTEM lib/uartsensor/include)
set_target_properties(libUartSensor PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(libUartSensor PROPERTIES CXX_CPPCHECK "")

# ------------------------------------------------- gcem ------------------------------------------------- */

include_directories(SYSTEM lib/gcem/include)

# ######################################################################################################################
# Installed Packages
# ######################################################################################################################

find_package(Threads REQUIRED)

# ######################################################################################################################

# Specify output path for executable files
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin/Release)

add_subdirectory(src)

if(ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

option(ENABLE_UNITY "Enable Unity builds of projects" OFF)
if(ENABLE_UNITY)
  # Add for any project you want to apply unity builds for
  set_target_properties(${PROJECT_NAME_LOWERCASE} PROPERTIES UNITY_BUILD ON)
endif()

# ######################################################################################################################
# Copy compile_commands.json to root directory
# ######################################################################################################################

message(STATUS "Copying compile commands from ${CMAKE_BINARY_DIR}/compile_commands.json to ${CMAKE_CURRENT_LIST_DIR}")

# Copy to source directory
add_custom_target(copy-compile-commands ALL DEPENDS ${CMAKE_CURRENT_LIST_DIR}/compile_commands.json)
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_LIST_DIR}/compile_commands.json
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/compile_commands.json
          ${CMAKE_CURRENT_LIST_DIR}/compile_commands.json
  DEPENDS # Unlike "proper" targets like executables and libraries, custom command / target pairs will not set up source
          # file dependencies, so we need to list file explicitly here
          generate-compile-commands ${CMAKE_BINARY_DIR}/compile_commands.json)

# Generate the compilation commands. Necessary so cmake knows where it came from and if for some reason you delete it.
add_custom_target(generate-compile-commands DEPENDS ${CMAKE_BINARY_DIR}/compile_commands.json)
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/compile_commands.json COMMAND ${CMAKE_COMMAND} -B${CMAKE_BINARY_DIR}
                                                                            -S${CMAKE_SOURCE_DIR})

# if(NOT EXISTS ${CMAKE_BINARY_DIR}/compile_commands.json) message(SEND_ERROR "There is no compile_commands.json in the
# path ${CMAKE_BINARY_DIR}") elseif(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/compile_commands.json) message(SEND_ERROR
# "Copying the compile commands failed!") endif()
