CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(kinect_recorder)

IF(NOT DEFINED CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release Debug")
ENDIF()

ADD_EXECUTABLE(KinectRecorder
	"${CMAKE_SOURCE_DIR}/src/KinectRecorder"
	"${CMAKE_SOURCE_DIR}/src/io"
)
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/src")

FIND_PACKAGE(gflags REQUIRED)
IF(gflags_FOUND)
	INCLUDE_DIRECTORIES(${gflags_INCLUDE_DIRS})
	TARGET_LINK_LIBRARIES(KinectRecorder gflags)
ENDIF()

FIND_PACKAGE(freenect2 REQUIRED)
IF(freenect2_FOUND)
	INCLUDE_DIRECTORIES(${freenect2_INCLUDE_DIR})
	TARGET_LINK_LIBRARIES(KinectRecorder ${freenect2_LIBRARIES})
ENDIF()

#FIND_PACKAGE(libjpeg-turbo REQUIRED)  # setup it manualy if not found
IF(libjpeg-turbo_FOUND)
	INCLUDE_DIRECTORIES(${libjpeg-turbo_INCLUDE_DIR})
	TARGET_LINK_LIBRARIES(KinectRecorder ${libjpeg-turbo_LIBRARIES})
ENDIF()

#FIND_PACKAGE(libpng REQUIRED)	 # setup it manualy if not found
IF(libpng_FOUND)
	INCLUDE_DIRECTORIES(${libpng_INCLUDE_DIR})
	TARGET_LINK_LIBRARIES(KinectRecorder ${libpng_LIBRARIES})
ENDIF()


# libjpeg-turbo DLLs have to be in path
# freenect2 DLLs have to be in path
# libpng DLLs have to be in path