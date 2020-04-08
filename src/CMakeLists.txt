message(STATUS "Building ${CMAKE_PROJECT_NAME}")

# Search all .cpp files
file (GLOB_RECURSE SOURCE_FILES "*.cpp")
# Remove gui folder
set (EXCLUDE_DIR "/gui/")
foreach (TMP_PATH ${SOURCE_FILES})
    string (FIND ${TMP_PATH} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)
    if (NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
        list (REMOVE_ITEM SOURCE_FILES ${TMP_PATH})
    endif ()
endforeach(TMP_PATH)

add_executable(${PROJECT_NAME_LOWERCASE}
                    ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME_LOWERCASE}
                PRIVATE project_options project_warnings
                        CONAN_PKG::fmt CONAN_PKG::spdlog
                        CONAN_PKG::boost CONAN_PKG::eigen
                        Threads::Threads
                        libvncxx libubcxx)