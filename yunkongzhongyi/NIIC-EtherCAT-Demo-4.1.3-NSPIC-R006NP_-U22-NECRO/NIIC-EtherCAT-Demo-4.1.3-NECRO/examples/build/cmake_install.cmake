# Install script for directory: /home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/api/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/c/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/c-exipc/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/c-io/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx-2master/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx-5master/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx-addr-mode/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx-mini/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx-multi-master/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx-restart/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx-sdo-request/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/general/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/general-array/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/xmlParser/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/qt-exipc/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx-exipc/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx-auto-register/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx-5master-ZCAI/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/ros_ethercat/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/general-xhand/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/general-encos/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/cxx-2master-encos/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/Ti5-Encos-Single/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/Ti5-Encos-2master/cmake_install.cmake")
  include("/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/Ti5-Encos-3master/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/niic/427/427/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
