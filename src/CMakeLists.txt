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
    PRIVATE project_options
            project_warnings
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

  if(NOT APPLE)
    target_link_libraries(${PROJECT_NAME_LOWERCASE} PRIVATE libnavio)
  endif()

  target_compile_definitions(${PROJECT_NAME_LOWERCASE} PUBLIC LOG_LEVEL=LOG_LEVEL_${LOG_LEVEL})
endif()
