# This file is part of INSTINCT, the INS Toolkit for Integrated
# Navigation Concepts and Training by the Institute of Navigation of
# the University of Stuttgart, Germany.
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.15)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(cmake/utils.cmake)
extract_project_version()

# Set the project name, version and language
project(
  "INSTINCT"
  VERSION ${INSTINCT_PROJECT_VERSION}
  LANGUAGES CXX)
string(TOLOWER ${CMAKE_PROJECT_NAME} PROJECT_NAME_LOWERCASE)
message(STATUS "${CMAKE_PROJECT_NAME}: ${INSTINCT_PROJECT_VERSION}")

# ######################################################################################################################

# Stop generation if compiler was changed
if((CMAKE_CXX_COMPILER_ID MATCHES ".*Clang" AND NOT $ENV{CC} MATCHES ".*clang") OR (CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
                                                                                    AND NOT $ENV{CC} MATCHES ".*gcc"))
  message(
    FATAL_ERROR
      "The Compiler was changed! Build directory is ${CMAKE_CXX_COMPILER_ID} and environment CC=$ENV{CC}. Please clean the build files and run cmake again."
  )
endif()

# ######################################################################################################################

# Export the SDKROOT Environment Variable for MacOS
if(APPLE)
  if(NOT DEFINED ENV{SDKROOT})
    execute_process(COMMAND xcrun --show-sdk-path OUTPUT_VARIABLE sdkpath)
    string(REGEX REPLACE "\n$" "" sdkpath "${sdkpath}")
    set(ENV{SDKROOT} ${sdkpath})
    message(STATUS "SDK-Path: $ENV{SDKROOT}")
  endif()
elseif(WIN32)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

# ######################################################################################################################

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_20)

# Basic settings
include(cmake/StandardProjectSettings.cmake)

# Prevent cmake builds with source dir .
include(cmake/PreventInSourceBuilds.cmake)

if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
  option(ENABLE_BUILD_WITH_TIME_TRACE "Enable -ftime-trace to generate time tracing .json files on clang" OFF)
  if(ENABLE_BUILD_WITH_TIME_TRACE)
    target_compile_options(project_options INTERFACE -ftime-trace)
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
# Resources
# ######################################################################################################################

include(cmake/CMakeRC.cmake)
run_cmakerc()

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
  "${CMAKE_SOURCE_DIR}/lib/imgui/*.cpp" "${CMAKE_SOURCE_DIR}/lib/imgui/*.h"
  "${CMAKE_SOURCE_DIR}/lib/imgui/misc/cpp/*.cpp" "${CMAKE_SOURCE_DIR}/lib/imgui/misc/cpp/*.h")
add_library(imgui ${_imgui_Sources})
include_directories(SYSTEM "${CMAKE_SOURCE_DIR}/lib/imgui")
include_directories(SYSTEM "${CMAKE_SOURCE_DIR}/lib/imgui/misc/cpp")
target_link_libraries(imgui PRIVATE project_options)
target_compile_definitions(imgui PUBLIC "ImDrawIdx=unsigned int")
set_target_properties(imgui PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(imgui PROPERTIES CXX_CPPCHECK "")

# ------------------------------------------- imgui-node-editor ------------------------------------------ */

# Define IMGUI_NODE_EDITOR_ROOT_DIR pointing to project root directory
get_filename_component(IMGUI_NODE_EDITOR_ROOT_DIR ${CMAKE_SOURCE_DIR}/lib/imgui-node-editor ABSOLUTE CACHE)
# Point CMake where to look for module files.
list(APPEND CMAKE_MODULE_PATH ${IMGUI_NODE_EDITOR_ROOT_DIR}/misc/cmake-modules)
find_package(imgui REQUIRED)
find_package(imgui_node_editor REQUIRED)
target_link_libraries(imgui_node_editor PRIVATE project_options imgui)
include_directories(SYSTEM lib/imgui-node-editor)
# include_directories(SYSTEM lib/imgui-node-editor/external/imgui)
set_target_properties(imgui_node_editor PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(imgui_node_editor PROPERTIES CXX_CPPCHECK "")

# -------------------------------------------- ImGuiFileDialog ------------------------------------------- */

add_library(
  ImGuiFileDialog
  "${CMAKE_SOURCE_DIR}/lib/ImGuiFileDialog/ImGuiFileDialog.cpp"
  "${CMAKE_SOURCE_DIR}/lib/ImGuiFileDialog/ImGuiFileDialog.h"
  "${CMAKE_SOURCE_DIR}/lib/ImGuiFileDialog/ImGuiFileDialogConfig.h")

if(WIN32)
  target_include_directories(ImGuiFileDialog PUBLIC "${CMAKE_SOURCE_DIR}/lib/ImGuiFileDialog/dirent/include")
endif()

include_directories(SYSTEM lib/ImGuiFileDialog)
target_link_libraries(ImGuiFileDialog PRIVATE project_options)
set_target_properties(ImGuiFileDialog PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(ImGuiFileDialog PROPERTIES CXX_CPPCHECK "")

# ------------------------------------------------ implot ------------------------------------------------ */

file(
  GLOB _implot_Sources
  LIST_DIRECTORIES OFF
  "${CMAKE_SOURCE_DIR}/lib/implot/*.cpp" "${CMAKE_SOURCE_DIR}/lib/implot/*.h")
add_library(implot ${_implot_Sources})
include_directories(SYSTEM "${CMAKE_SOURCE_DIR}/lib/implot")
target_link_libraries(implot PRIVATE project_options)
target_compile_definitions(implot PUBLIC "ImDrawIdx=unsigned int")
set_target_properties(implot PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(implot PROPERTIES CXX_CPPCHECK "")

# ---------------------------------------------- Application --------------------------------------------- */

add_subdirectory(lib/application)
include_directories(SYSTEM lib/application/include)
target_link_libraries(application PRIVATE project_options implot)
target_compile_definitions(application PUBLIC "_CONSOLE")
set_target_properties(application PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(application PROPERTIES CXX_CPPCHECK "")

# ----------------------------------------------- VectorNav ---------------------------------------------- */

add_subdirectory(lib/vnproglib/cpp libvncxx)
include_directories(SYSTEM lib/vnproglib/cpp/include)
set_target_properties(libvncxx PROPERTIES CXX_CLANG_TIDY "")
set_target_properties(libvncxx PROPERTIES CXX_CPPCHECK "")

# ------------------------------------------------ Navio2 ------------------------------------------------ */

if(NOT APPLE AND NOT WIN32)
  # Include navio2 library
  add_library(libnavio lib/navio2/C++/Navio/Common/gpio.cpp
                       lib/navio2/C++/Navio/Common/I2Cdev.cpp
                       lib/navio2/C++/Navio/Common/MPU9250.cpp
                       lib/navio2/C++/Navio/Common/MS5611.cpp
                       lib/navio2/C++/Navio/Common/Ublox.cpp
                       lib/navio2/C++/Navio/Common/Util.cpp
                       lib/navio2/C++/Navio/Navio2/ADC_Navio2.cpp
                       lib/navio2/C++/Navio/Navio2/Led_Navio2.cpp
                       lib/navio2/C++/Navio/Navio2/LSM9DS1.cpp
                       lib/navio2/C++/Navio/Navio2/PWM.cpp
                       lib/navio2/C++/Navio/Navio2/RCInput_Navio2.cpp
                       lib/navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp
                       lib/navio2/C++/Navio/Navio2/RGBled.cpp
                       lib/navio2/C++/Navio/Common/ADC.h
                       lib/navio2/C++/Navio/Common/gpio.h
                       lib/navio2/C++/Navio/Common/I2Cdev.h
                       lib/navio2/C++/Navio/Common/InertialSensor.h
                       lib/navio2/C++/Navio/Common/Led.h
                       lib/navio2/C++/Navio/Common/MPU9250.h
                       lib/navio2/C++/Navio/Common/MS5611.h
                       lib/navio2/C++/Navio/Common/RCInput.h
                       lib/navio2/C++/Navio/Common/RCOutput.h
                       lib/navio2/C++/Navio/Common/SPIdev.h
                       lib/navio2/C++/Navio/Common/Ublox.h
                       lib/navio2/C++/Navio/Common/Util.h
                       lib/navio2/C++/Navio/Navio2/ADC_Navio2.h
                       lib/navio2/C++/Navio/Navio2/Led_Navio2.h
                       lib/navio2/C++/Navio/Navio2/LSM9DS1.h
                       lib/navio2/C++/Navio/Navio2/PWM.h
                       lib/navio2/C++/Navio/Navio2/RCInput_Navio2.h
                       lib/navio2/C++/Navio/Navio2/RCOutput_Navio2.h
                       lib/navio2/C++/Navio/Navio2/RGBled.h)
  target_include_directories(libnavio SYSTEM PRIVATE lib/navio2/C++/Navio)
  include_directories(SYSTEM lib/navio2/C++
                             lib/navio2/C++/Navio)
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
if(NOT MSVC OR NOT CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin/Debug)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin/Release)
else()
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin/Release) # it always takes the DEBUG path (MSVC...)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin/Release)
endif()

set(INSTINCT_SOURCE_DIR ${CMAKE_SOURCE_DIR})
set(INSTINCT_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)

add_subdirectory(src)

add_subdirectory(doc)

if(ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

option(ENABLE_UNITY "Enable Unity builds of projects" OFF)
if(ENABLE_UNITY)
  # Add for any project you want to apply unity builds for
  set_target_properties(${PROJECT_NAME_LOWERCASE} PROPERTIES UNITY_BUILD ON)
endif()

# Because MSVC sucks and we need to tell the linker to link the release versions manually
if(MSVC AND CMAKE_BUILD_TYPE STREQUAL "Release")
  set_target_properties(
    ${PROJECT_NAME_LOWERCASE}
    instinct-resources
    imgui
    imgui_node_editor
    ImGuiFileDialog
    implot
    application
    libvncxx
    libUartSensor
    PROPERTIES MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
endif()

# ######################################################################################################################
# Copy compile_commands.json to root directory
# ######################################################################################################################

if(NOT WIN32)
  message(STATUS "Copying compile commands from ${CMAKE_BINARY_DIR}/compile_commands.json to ${CMAKE_CURRENT_LIST_DIR}")

  # Copy to source directory
  add_custom_target(copy-compile-commands ALL DEPENDS ${CMAKE_CURRENT_LIST_DIR}/compile_commands.json)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_LIST_DIR}/compile_commands.json
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_CURRENT_LIST_DIR}/compile_commands.json
    DEPENDS # Unlike "proper" targets like executables and libraries, custom command / target pairs will not set up
            # source file dependencies, so we need to list file explicitly here
            generate-compile-commands ${CMAKE_BINARY_DIR}/compile_commands.json)

  # Generate the compilation commands. Necessary so cmake knows where it came from and if for some reason you delete it.
  add_custom_target(generate-compile-commands DEPENDS ${CMAKE_BINARY_DIR}/compile_commands.json)
  add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/compile_commands.json COMMAND ${CMAKE_COMMAND} -B${CMAKE_BINARY_DIR}
                                                                              -S${CMAKE_SOURCE_DIR})

  # if(NOT EXISTS ${CMAKE_BINARY_DIR}/compile_commands.json) message(SEND_ERROR "There is no compile_commands.json in
  # the path ${CMAKE_BINARY_DIR}") elseif(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/compile_commands.json) message(SEND_ERROR
  # "Copying the compile commands failed!") endif()
endif()
