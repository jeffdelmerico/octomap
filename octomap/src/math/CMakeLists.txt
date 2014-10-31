SET (octomath_stereo_SRCS
  Vector3.cpp
  Quaternion.cpp
  Pose6D.cpp
 )


ADD_LIBRARY( octomath_stereo SHARED ${octomath_stereo_SRCS})

SET_TARGET_PROPERTIES( octomath_stereo PROPERTIES
  VERSION ${OCTOMAP_STEREO_VERSION}
  SOVERSION ${OCTOMAP_STEREO_SOVERSION}
  INSTALL_NAME_DIR ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}   # this seems to be necessary for MacOS X
)
# INSTALL_NAME_DIR seems to be necessary for MacOS X

ADD_LIBRARY( octomath_stereo-static STATIC ${octomath_stereo_SRCS})
SET_TARGET_PROPERTIES(octomath_stereo-static PROPERTIES OUTPUT_NAME "octomath_stereo")

install(TARGETS octomath_stereo octomath_stereo-static ${INSTALL_TARGETS_DEFAULT_ARGS})
 
