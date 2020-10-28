# Search all .cpp files
file(GLOB_RECURSE SOURCE_FILES "*.cpp")
# Remove gui folder
set(EXCLUDE_DIR "/gui/")
foreach(TMP_PATH ${SOURCE_FILES})
  string(FIND ${TMP_PATH} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)
  if(NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
    list(REMOVE_ITEM SOURCE_FILES ${TMP_PATH})
  endif()
endforeach(TMP_PATH)

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
            Threads::Threads
            libvncxx
            libUartSensor)

  if(NOT APPLE)
    target_link_libraries(${PROJECT_NAME_LOWERCASE} PRIVATE libnavio)
  endif()

  target_compile_definitions(${PROJECT_NAME_LOWERCASE} PUBLIC LOG_LEVEL=LOG_LEVEL_${LOG_LEVEL})
endif()

if(ENABLE_GUI)
  message(STATUS "Building ${CMAKE_PROJECT_NAME}-GUI")

  # Search all .cpp files
  file(GLOB_RECURSE SOURCE_FILES_GUI "*.cpp")
  # this is the file I want to exclude / remove from the list
  get_filename_component(full_path_test_cpp ${CMAKE_SOURCE_DIR}/src/main.cpp ABSOLUTE)
  list(REMOVE_ITEM SOURCE_FILES_GUI "${full_path_test_cpp}")
  # message("${SOURCE_FILES_GUI}")

  set(CMAKE_AUTOMOC ON)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5Core REQUIRED)

  add_executable("${PROJECT_NAME_LOWERCASE}-gui" ${SOURCE_FILES_GUI})

  target_include_directories("${PROJECT_NAME_LOWERCASE}-gui" SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/lib/nodeeditor/include)

  target_link_libraries(
    "${PROJECT_NAME_LOWERCASE}-gui"
    PRIVATE project_options
            project_warnings
            CONAN_PKG::fmt
            CONAN_PKG::spdlog
            NodeEditor::nodes
            CONAN_PKG::boost
            CONAN_PKG::eigen
            Threads::Threads
            libvncxx
            libUartSensor)

  if(NOT APPLE)
    target_link_libraries("${PROJECT_NAME_LOWERCASE}-gui" PRIVATE libnavio)
  endif()

  target_compile_definitions("${PROJECT_NAME_LOWERCASE}-gui" PUBLIC LOG_LEVEL=LOG_LEVEL_${LOG_LEVEL})

  set_target_properties("${PROJECT_NAME_LOWERCASE}-gui" PROPERTIES CXX_CLANG_TIDY "")
endif()
