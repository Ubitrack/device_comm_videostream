# All cpp files for UTCORE
FILE(GLOB UBITRACK_SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
        )

foreach(fpath ${UBITRACK_SRCS})
    GET_FILENAME_COMPONENT(fname ${fpath} NAME_WE)
    set(UBITRACK_LIBRARY_NAME ${fname})

  add_library(${UBITRACK_LIBRARY_NAME} SHARED ${fpath})

  target_link_libraries(${UBITRACK_LIBRARY_NAME}
      CONAN_PKG::ubitrack_core
      CONAN_PKG::ubitrack_vision
      CONAN_PKG::ubitrack_dataflow
      )
  if (WITH_NVENC)
    target_link_libraries(${UBITRACK_LIBRARY_NAME}
      CONAN_PKG::nvpipe
      )
  endif(WITH_NVENC)

  if (WITH_NVENC_RTSP)
    target_link_libraries(${UBITRACK_LIBRARY_NAME}
      CONAN_PKG::nvenc_rtsp
      )
   target_include_directories(${UBITRACK_LIBRARY_NAME}
           PRIVATE ${CUDA_INCLUDE_DIRS}
           )
  endif(WITH_NVENC_RTSP)

  if (WITH_NDI)
    target_link_libraries(${UBITRACK_LIBRARY_NAME}
            CONAN_PKG::newtekndi
            )
#    target_link_libraries(${UBITRACK_LIBRARY_NAME}
#            ${NDI_LIBRARIES}
#            )
#    target_include_directories(${UBITRACK_LIBRARY_NAME}
#            PRIVATE ${NDI_INCLUDE_DIR}
#            )
  endif(WITH_NDI)

  # do not create symlinks to versioned names of component libraries
  set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES NO_SONAME ON)

  # Apply Target Properties
  if(MSVC)
    if(CMAKE_CROSSCOMPILING)
      set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
    endif()
    set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
    set(UBITRACK_COMPILE_FLAGS ${UBITRACK_COMPILE_FLAGS} "/bigobj ")
  endif()

  ubitrack_set_target_properties(${UBITRACK_LIBRARY_NAME})
  ubitrack_install_component(${UBITRACK_LIBRARY_NAME})
endforeach()

FILE(GLOB UBITRACK_HEADERS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
        )
foreach(hdr ${UBITRACK_HEADERS})
  string(REGEX REPLACE "${CMAKE_BINARY_DIR}/" "" hdr2 "${hdr}")
  GET_FILENAME_COMPONENT(fpath ${hdr2} PATH)
  IF(fpath)
    install(FILES ${hdr} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/utComponents/${fpath}" COMPONENT dev)
  ELSE(fpath)
    install(FILES ${hdr} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/utComponents" COMPONENT dev)
  ENDIF(fpath)
endforeach()
