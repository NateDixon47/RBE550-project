# Install script for directory: /home/nddixon/RBE_Grad/project_ws/src/warehouse_simulation_toolkit

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nddixon/RBE_Grad/project_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nddixon/RBE_Grad/project_ws/build/warehouse_simulation_toolkit/catkin_generated/installspace/warehouse_simulation_toolkit.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/warehouse_simulation_toolkit/cmake" TYPE FILE FILES
    "/home/nddixon/RBE_Grad/project_ws/build/warehouse_simulation_toolkit/catkin_generated/installspace/warehouse_simulation_toolkitConfig.cmake"
    "/home/nddixon/RBE_Grad/project_ws/build/warehouse_simulation_toolkit/catkin_generated/installspace/warehouse_simulation_toolkitConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/warehouse_simulation_toolkit" TYPE FILE FILES "/home/nddixon/RBE_Grad/project_ws/src/warehouse_simulation_toolkit/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/warehouse_simulation_toolkit" TYPE PROGRAM FILES "/home/nddixon/RBE_Grad/project_ws/build/warehouse_simulation_toolkit/catkin_generated/installspace/global_planner.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/warehouse_simulation_toolkit/launch" TYPE DIRECTORY FILES "/home/nddixon/RBE_Grad/project_ws/src/warehouse_simulation_toolkit/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/warehouse_simulation_toolkit/maps" TYPE DIRECTORY FILES "/home/nddixon/RBE_Grad/project_ws/src/warehouse_simulation_toolkit/maps/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/warehouse_simulation_toolkit/param" TYPE DIRECTORY FILES "/home/nddixon/RBE_Grad/project_ws/src/warehouse_simulation_toolkit/param/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/warehouse_simulation_toolkit/rviz" TYPE DIRECTORY FILES "/home/nddixon/RBE_Grad/project_ws/src/warehouse_simulation_toolkit/rviz/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/warehouse_simulation_toolkit" TYPE FILE FILES "/home/nddixon/RBE_Grad/project_ws/src/warehouse_simulation_toolkit/plugin_description.xml")
endif()

