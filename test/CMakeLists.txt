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

# Link all files in the Source directory
include_directories(${CMAKE_SOURCE_DIR}/src)
add_executable(
  tests
  util/InsTimeTests.cpp
  util/InsMechanizationTests.cpp
  util/InsTransformationsTests.cpp
  util/LinearAlgebraTests.cpp
  util/LoggerTests.cpp
  NodeData/State/StateDataTests.cpp
  Nodes/DataLogger/IMU/VectorNavDataLoggerTest.cpp
  Nodes/Integrator/ImuIntegratorTest.cpp
  ${SRC_FILES})

target_link_libraries(
  tests
  PRIVATE project_warnings
          project_options
          catch_main
          CONAN_PKG::fmt
          CONAN_PKG::spdlog
          CONAN_PKG::boost
          CONAN_PKG::eigen
          Threads::Threads
          CONAN_PKG::nlohmann_json
          imgui
          imgui_node_editor
          ImGuiFileDialog
          implot
          application
          libvncxx
          libUartSensor)

if(NOT APPLE)
  target_link_libraries(tests PRIVATE libnavio)
endif()

target_compile_definitions(tests PUBLIC LOG_LEVEL=LOG_LEVEL_${LOG_LEVEL})

# automatically discover tests that are defined in catch based test files you can modify the unittests. TEST_PREFIX to
# whatever you want, or use different for different binaries catch_discover_tests(tests)
catch_discover_tests(
  tests
  REPORTER
  junit
  OUTPUT_DIR
  .
  OUTPUT_SUFFIX
  .xml)
