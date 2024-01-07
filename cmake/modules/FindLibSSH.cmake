# - Try to find LibSSH
# Once done this will define
#
#  LIBSSH_FOUND - system has LibSSH
#  LIBSSH_INCLUDE_DIRS - the LibSSH include directory
#  LIBSSH_LIBRARIES - Link these to use LibSSH
#  LIBSSH_DEFINITIONS - Compiler switches required for using LibSSH
#
#  Copyright (c) 2009 Andreas Schneider <mail@cynapses.org>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if(NOT (LIBSSH_INCLUDE_DIR AND LIBSSH_LIBRARY))
  find_package(PkgConfig)
  pkg_check_modules(PC_LIBSSH QUIET libssh)

  find_path(LIBSSH_INCLUDE_DIR
    NAMES
      libssh/libssh.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
      ${CMAKE_INCLUDE_PATH}
      ${CMAKE_INSTALL_PREFIX}/include
  )

  find_library(LIBSSH_LIBRARY
    NAMES
      ssh
      libssh
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
      ${CMAKE_LIBRARY_PATH}
      ${CMAKE_INSTALL_PREFIX}/lib
  )

  mark_as_advanced(LIBSSH_INCLUDE_DIR LIBSSH_LIBRARY)
endif(NOT (LIBSSH_INCLUDE_DIR AND LIBSSH_LIBRARY))

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBSSH
    FOUND_VAR LIBSSH_FOUND
    REQUIRED_VARS LIBSSH_LIBRARY LIBSSH_INCLUDE_DIR)

if(LIBSSH_FOUND)
  set(LIBSSH_LIBRARIES ${LIBSSH_LIBRARY})
  set(LIBSSH_INCLUDE_DIRS ${LIBSSH_INCLUDE_DIR})
  set(LIBSSH_DEFINITIONS ${PC_LIBSSH_CFLAGS_OTHER})
endif()

if(LIBSSH_FOUND AND NOT TARGET LIBSSH::LIBSSH)
  add_library(LIBSSH::LIBSSH UNKNOWN IMPORTED)
  set_target_properties(
      LIBSSH::LIBSSH PROPERTIES
      IMPORTED_LOCATION "${LIBSSH_LIBRARY}"
      INTERFACE_COMPILE_OPTIONS "${PC_LIBSSH_CFLAGS_OTHER}"
      INTERFACE_INCLUDE_DIRECTORIES "${LIBSSH_INCLUDE_DIR}")
endif()


