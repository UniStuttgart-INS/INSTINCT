# This file is part of INSTINCT, the INS Toolkit for Integrated
# Navigation Concepts and Training by the Institute of Navigation of
# the University of Stuttgart, Germany.
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md


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
  PRIVATE project_warnings)
target_link_libraries(
  tests
  PRIVATE project_options
          Catch2::Catch2WithMain
          instinct::rc
          fmt::fmt
          spdlog::spdlog
          Boost::program_options
          Eigen3::Eigen
          nlohmann_json::nlohmann_json
          unordered_dense::unordered_dense
          muparser::muparser
          Threads::Threads
          imgui
          imgui_node_editor
          ImGuiFileDialog
          implot
          application
          libvncxx
          libUartSensor)

target_compile_definitions(tests PUBLIC IMGUI_DEFINE_MATH_OPERATORS)

if(NOT APPLE AND NOT WIN32)
  target_link_libraries(tests PRIVATE libnavio)
endif()

if(ENABLE_GPERFTOOLS)
  if(GPERFTOOLS_FOUND)
    message(STATUS "Linking Gperftools profiler")
    target_link_libraries(tests PRIVATE profiler)
  else()
    message(WARNING "Gperftools profiler is enabled but was not found. Not using it")
  endif()
endif()

target_compile_definitions(tests PUBLIC JSON_DIAGNOSTICS=1)

target_compile_definitions(tests PUBLIC LOG_LEVEL=LOG_LEVEL_${LOG_LEVEL})

if(APPLE)
  target_compile_definitions(tests PUBLIC BOOST_ASIO_HAS_STD_INVOKE_RESULT=1)
endif()

target_compile_definitions(tests PUBLIC TESTING=1)

# automatically discover tests that are defined in catch based test files you can modify the unittests. TEST_PREFIX to
# whatever you want, or use different for different binaries

option(CATCH2_JUNIT_OUTPUT "Generate Junit output from tests instead of command-line output" FALSE)

include(CTest)
include(Catch)

if(NOT WIN32 AND CATCH2_JUNIT_OUTPUT)
  catch_discover_tests(tests
                       WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                       REPORTER junit
                       OUTPUT_DIR ${CMAKE_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE}/test
                       OUTPUT_SUFFIX .xml)
else()
  catch_discover_tests(tests
                       WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()
