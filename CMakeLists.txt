cmake_minimum_required(VERSION 3.8.0)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(ubitrack_device_comm_videostream)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include(GNUInstallDirs)
include(cmake/UbitrackConfig)

if(WITH_NVENC)
  message(STATUS "NVEnc is enabled.")
  set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_NVENC")
endif(WITH_NVENC)

if(WITH_NDI)
  find_package(NDI REQUIRED)
  message(STATUS "Found NDI: ${NDI_INCLUDE_DIR}  ${NDI_LIBRARIES}")
  set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_NDI")

endif(WITH_NDI)

# currently this will not build if compiled without nvenc ..
add_subdirectory(src/videostream)

ubitrack_install_utql("doc/utql/*.xml" "doc/utql/*/*.xml"  "doc/utql/*/*/*.xml")
ubitrack_install_doc("doc/utqlDoc/*" "doc/utqlDoc/*/*" "doc/utqlDoc/*/*/*")
