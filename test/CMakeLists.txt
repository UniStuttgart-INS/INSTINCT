# automatically enable catch2 to generate ctest targets
if(CONAN_CATCH2_ROOT_DEBUG)
  include(${CONAN_CATCH2_ROOT_DEBUG}/lib/cmake/Catch2/Catch.cmake)
else()
  include(${CONAN_CATCH2_ROOT}/lib/cmake/Catch2/Catch.cmake)
endif()

add_library(catch_main STATIC catch_main.cpp)
target_link_libraries(catch_main PUBLIC CONAN_PKG::catch2)
target_link_libraries(catch_main PRIVATE project_options)

# Link all files in the Source directory
include_directories(${CMAKE_SOURCE_DIR}/src)
add_executable(
  tests
  util/InsMechanizationTests.cpp
  util/InsTimeTests.cpp
  util/InsUtilTests.cpp
  util/LoggerTests.cpp
  Nodes/DataLogger/IMU/VectorNavDataLoggerTest.cpp
  ${CMAKE_SOURCE_DIR}/src/util/Logger.cpp
  ${CMAKE_SOURCE_DIR}/src/util/InsMechanization.cpp
  ${CMAKE_SOURCE_DIR}/src/util/InsTime.cpp
  ${CMAKE_SOURCE_DIR}/src/util/InsUtil.cpp
  ${CMAKE_SOURCE_DIR}/src/Nodes/Node.cpp
  ${CMAKE_SOURCE_DIR}/src/Nodes/DataProvider/IMU/Imu.cpp
  ${CMAKE_SOURCE_DIR}/src/Nodes/DataProvider/IMU/ImuFileReader.cpp
  ${CMAKE_SOURCE_DIR}/src/Nodes/DataProvider/IMU/FileReader/VectorNavFile.cpp
  ${CMAKE_SOURCE_DIR}/src/Nodes/DataProvider/Protocol/FileReader.cpp
  ${CMAKE_SOURCE_DIR}/src/Nodes/DataLogger/DataLogger.cpp
  ${CMAKE_SOURCE_DIR}/src/Nodes/DataLogger/IMU/VectorNavDataLogger.cpp)

target_link_libraries(
  tests
  PRIVATE project_warnings
          project_options
          catch_main
          CONAN_PKG::fmt
          CONAN_PKG::spdlog
          CONAN_PKG::boost
          CONAN_PKG::eigen
          Threads::Threads)

target_compile_definitions(tests PUBLIC LOG_LEVEL=LOG_LEVEL_${LOG_LEVEL})

if(NOT ENABLE_SENSORS)
  target_compile_definitions(tests PUBLIC DISABLE_SENSORS)
endif()

# automatically discover tests that are defined in catch based test files you can modify the unittests. TEST_PREFIX to
# whatever you want, or use different for different binaries
catch_discover_tests(tests)
# TEST_PREFIX "unittests." EXTRA_ARGS -s --reporter=xml --out=tests.xml)
