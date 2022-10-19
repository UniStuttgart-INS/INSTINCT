# This file is part of INSTINCT, the INS Toolkit for Integrated
# Navigation Concepts and Training by the Institute of Navigation of
# the University of Stuttgart, Germany.
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md

# automatically enable catch2 to generate ctest targets
if(CONAN_CATCH2_ROOT_DEBUG)
  include(${CONAN_CATCH2_ROOT_DEBUG}/lib/cmake/Catch2/Catch.cmake)
else()
  include(${CONAN_CATCH2_ROOT}/lib/cmake/Catch2/Catch.cmake)
endif()

add_library(catch_main STATIC catch_main.cpp)
target_link_libraries(catch_main PUBLIC CONAN_PKG::catch2)
target_link_libraries(catch_main PRIVATE project_options)

# Remove main.cpp
foreach(TMP_PATH ${SRC_FILES})
  string(FIND ${TMP_PATH} "main.cpp" EXCLUDE_DIR_FOUND)
  if(NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
    list(REMOVE_ITEM SRC_FILES ${TMP_PATH})
  endif()
endforeach(TMP_PATH)

# Search all .cpp files
file(GLOB_RECURSE TEST_SOURCE_FILES "src/*.cpp")

# Link all files in the Source directory
add_executable(
  tests
  ${TEST_SOURCE_FILES}
  ${SRC_FILES})

target_include_directories(tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(tests PRIVATE src)

target_link_libraries(
  tests
  PRIVATE project_warnings
          project_options
          catch_main
          instinct::rc
          CONAN_PKG::fmt
          CONAN_PKG::spdlog
          CONAN_PKG::boost
          CONAN_PKG::eigen
          CONAN_PKG::nlohmann_json
          Threads::Threads
          imgui
          imgui_node_editor
          ImGuiFileDialog
          implot
          application
          libvncxx)

if(ENABLE_UARTSENSORS)
  target_compile_definitions(tests PUBLIC HAS_UARTSENSOR_LIBRARY=1)
  target_link_libraries(tests PRIVATE libUartSensor)
endif()

if(NOT APPLE AND NOT WIN32)
  target_link_libraries(tests PRIVATE libnavio)
endif()

target_compile_definitions(tests PUBLIC LOG_LEVEL=LOG_LEVEL_${LOG_LEVEL})

if(APPLE)
  target_compile_definitions(tests PUBLIC BOOST_ASIO_HAS_STD_INVOKE_RESULT=1)
endif()

target_compile_definitions(tests PUBLIC TESTING=1)

# automatically discover tests that are defined in catch based test files you can modify the unittests. TEST_PREFIX to
# whatever you want, or use different for different binaries

option(CATCH2_JUNIT_OUTPUT "Generate Junit output from tests instead of command-line output" FALSE)

if(NOT WIN32 AND CATCH2_JUNIT_OUTPUT)
  catch_discover_tests(
    tests
    WORKING_DIRECTORY
    ${CMAKE_SOURCE_DIR}
    REPORTER
    junit
    OUTPUT_DIR
    ${CMAKE_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE}/test
    OUTPUT_SUFFIX
    .xml)
else()
  catch_discover_tests(tests WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()
