# Install script for directory: /workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/workspaces/ros2_jazzy_docker/ros2_ws/install/stage")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RELEASE")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stage/worlds" TYPE FILE FILES
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/amcl-sonar.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/autolab.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/camera.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/everything.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/lsp_test.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/mbicp.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/nd.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/roomba.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/simple.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/test.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/uoa_robotics_lab.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/vfh.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/wavefront-remote.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/wavefront.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/wifi.cfg"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/SFU.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/autolab.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/camera.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/circuit.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/everything.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/fasr.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/fasr2.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/fasr_plan.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/large.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/lsp_test.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/mbicp.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/pioneer_flocking.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/pioneer_follow.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/pioneer_walle.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/roomba.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/sensor_noise_demo.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/sensor_noise_module_demo.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/simple.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/uoa_robotics_lab.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/wifi.world"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/beacons.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/chatterbox.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/hokuyo.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/irobot.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/map.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/objects.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/pantilt.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/pioneer.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/sick.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/ubot.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/uoa_robotics_lab_models.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/walle.inc"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/cfggen.sh"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/test.sh"
    "/workspaces/ros2_jazzy_docker/ros2_ws/src/Stage/worlds/worldgen.sh"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/workspaces/ros2_jazzy_docker/ros2_ws/build/stage/worlds/benchmark/cmake_install.cmake")
  include("/workspaces/ros2_jazzy_docker/ros2_ws/build/stage/worlds/bitmaps/cmake_install.cmake")
  include("/workspaces/ros2_jazzy_docker/ros2_ws/build/stage/worlds/wifi/cmake_install.cmake")

endif()

