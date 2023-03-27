# Install script for directory: /home/george/FML_AutonomousCar/c_generated_code

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/george/FML_AutonomousCar/c_generated_code")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/george/FML_AutonomousCar/c_generated_code/libacados_ocp_solver_Dynamic_Bicycle.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/george/FML_AutonomousCar/c_generated_code/libacados_ocp_solver_Dynamic_Bicycle.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/george/FML_AutonomousCar/c_generated_code/libacados_ocp_solver_Dynamic_Bicycle.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/george/FML_AutonomousCar/c_generated_code/libacados_ocp_solver_Dynamic_Bicycle.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/george/FML_AutonomousCar/c_generated_code" TYPE SHARED_LIBRARY FILES "/home/george/FML_AutonomousCar/c_generated_code/build/libacados_ocp_solver_Dynamic_Bicycle.so")
  if(EXISTS "$ENV{DESTDIR}/home/george/FML_AutonomousCar/c_generated_code/libacados_ocp_solver_Dynamic_Bicycle.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/george/FML_AutonomousCar/c_generated_code/libacados_ocp_solver_Dynamic_Bicycle.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/george/FML_AutonomousCar/c_generated_code/libacados_ocp_solver_Dynamic_Bicycle.so"
         OLD_RPATH "/home/george/acados/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/george/FML_AutonomousCar/c_generated_code/libacados_ocp_solver_Dynamic_Bicycle.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/george/FML_AutonomousCar/c_generated_code/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
