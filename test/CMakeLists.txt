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
add_executable(
  tests
  src/FlowTester.cpp
  src/internal/CallbackManagerTests.cpp
  src/internal/ConfigManagerTests.cpp
  src/Navigation/INS/MechanizationTests.cpp
  src/Navigation/Time/InsTimeTests.cpp
  src/Navigation/Transformations/CoordinateFramesTests.cpp
  src/NodeData/State/PosVelAttTests.cpp
  src/Nodes/DataLogger/IMU/VectorNavDataLoggerTests.cpp
  src/util/Container/ScrollingBufferTests.cpp
  src/util/UartSensors/Ublox/UbloxTypesTests.cpp
  src/util/UartSensors/Ublox/UbloxUtilitiesTests.cpp
  src/util/LoggerTests.cpp
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
          libvncxx
          libUartSensor)

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

# if(NOT WIN32)
#   catch_discover_tests(
#     tests
#     WORKING_DIRECTORY
#     ${CMAKE_SOURCE_DIR}
#     REPORTER
#     junit
#     OUTPUT_DIR
#     ${CMAKE_SOURCE_DIR}/build/Debug/test
#     OUTPUT_SUFFIX
#     .xml)
# else()
  catch_discover_tests(tests WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
# endif()
