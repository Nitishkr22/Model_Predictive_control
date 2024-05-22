# Install script for directory: /home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/src/genesis_path_follower

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/install")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/genesis_path_follower/msg" TYPE FILE FILES
    "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/src/genesis_path_follower/msg/state_est.msg"
    "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/src/genesis_path_follower/msg/mpc_path.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/genesis_path_follower/cmake" TYPE FILE FILES "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/build/genesis_path_follower/catkin_generated/installspace/genesis_path_follower-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/devel/include/genesis_path_follower")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/devel/share/roseus/ros/genesis_path_follower")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/devel/share/common-lisp/ros/genesis_path_follower")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/devel/share/gennodejs/ros/genesis_path_follower")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/devel/lib/python2.7/dist-packages/genesis_path_follower")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/devel/lib/python2.7/dist-packages/genesis_path_follower")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/build/genesis_path_follower/catkin_generated/installspace/genesis_path_follower.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/genesis_path_follower/cmake" TYPE FILE FILES "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/build/genesis_path_follower/catkin_generated/installspace/genesis_path_follower-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/genesis_path_follower/cmake" TYPE FILE FILES
    "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/build/genesis_path_follower/catkin_generated/installspace/genesis_path_followerConfig.cmake"
    "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/build/genesis_path_follower/catkin_generated/installspace/genesis_path_followerConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/genesis_path_follower" TYPE FILE FILES "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/src/genesis_path_follower/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/genesis_path_follower" TYPE DIRECTORY FILES "/home/radar/Documents/MPC/Model_Predictive_control/gensis_mpc/src/genesis_path_follower/launch")
endif()

