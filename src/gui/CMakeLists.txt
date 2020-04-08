message(STATUS "Building ${CMAKE_PROJECT_NAME}-GUI")

file(GLOB SRCFILES    
    "*.cpp"
)

set(CMAKE_AUTOMOC ON)
set (CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

add_executable("${PROJECT_NAME_LOWERCASE}-gui"
                    ${SRCFILES})

target_include_directories("${PROJECT_NAME_LOWERCASE}-gui"
                PRIVATE ${CMAKE_SOURCE_DIR}/lib/nodeeditor/include)

target_link_libraries("${PROJECT_NAME_LOWERCASE}-gui"
                PRIVATE project_options project_warnings
                        CONAN_PKG::fmt CONAN_PKG::spdlog
                        NodeEditor::nodes)

