# This file is part of INSTINCT, the INS Toolkit for Integrated
# Navigation Concepts and Training by the Institute of Navigation of
# the University of Stuttgart, Germany.
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# Search all .cpp files
file(GLOB_RECURSE SOURCE_FILES "*.cpp")

set(SRC_FILES
    "${SOURCE_FILES}"
    PARENT_SCOPE)

if(ENABLE_MAIN)
  message(STATUS "Building ${CMAKE_PROJECT_NAME}")

  add_executable(${PROJECT_NAME_LOWERCASE} ${SOURCE_FILES})

  target_link_libraries(
    ${PROJECT_NAME_LOWERCASE}
    PRIVATE project_warnings)
  target_link_libraries(
    ${PROJECT_NAME_LOWERCASE}
    PRIVATE project_options
            instinct::rc
            fmt::fmt
            spdlog::spdlog
            Boost::program_options
            Eigen3::Eigen
            nlohmann_json::nlohmann_json
            unordered_dense::unordered_dense
            Threads::Threads
            imgui
            imgui_node_editor
            ImGuiFileDialog
            implot
            application
            ssh
            libvncxx
            libUartSensor)

  if(NOT APPLE AND NOT WIN32)
    target_link_libraries(${PROJECT_NAME_LOWERCASE} PRIVATE libnavio)
  endif()

  if(ENABLE_GPERFTOOLS)
    if(GPERFTOOLS_FOUND)
      message(STATUS "Linking Gperftools profiler")
      target_link_libraries(${PROJECT_NAME_LOWERCASE} PRIVATE profiler)
    else()
      message(WARNING "Gperftools profiler is enabled but was not found. Not using it")
    endif()
  endif()


  target_compile_definitions(${PROJECT_NAME_LOWERCASE} PUBLIC LOG_LEVEL=LOG_LEVEL_${LOG_LEVEL})

  if(APPLE)
    target_compile_definitions(${PROJECT_NAME_LOWERCASE} PUBLIC BOOST_ASIO_HAS_STD_INVOKE_RESULT=1)
  endif()
endif()
