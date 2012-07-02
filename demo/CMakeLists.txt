cmake_minimum_required(VERSION 2.6)

SET( 	HEAD_DEMO
		../CRForestEstimator.cpp
		../CRTree.cpp
		gl_camera.cpp
		main.cpp 
)

SET(CMAKE_BUILD_TYPE "Release")

IF(WIN32)

	SET( OPENCV_INCLUDES E:\\Documents\\Code\\libraries\\OpenCV-2.3.0\\include ) 
	SET( OPENCV_LIB E:\\Documents\\Code\\libraries\\OpenCV-2.3.0\\lib ) 

	SET( OPENNI_INCLUDES E:\\Documents\\Code\\libraries\\OpenNI\\Include ) 
	SET( OPENNI_LIB E:\\Documents\\Code\\libraries\\OpenNI\\Lib ) 

	SET( FREEGLUT_LIB E:\\Documents\\Code\\libraries\\freeglut\\lib ) 
	LINK_DIRECTORIES( ${OPENNI_LIB} ${OPENCV_LIB} ${FREEGLUT_LIB} )


ELSE(WIN32)

	SET( OPENCV_INCLUDES /usr/local/include ) 
	SET( OPENCV_LIB /usr/local/lib ) 
	SET( OPENNI_INCLUDES /home/matthias/OpenNI/Include ) 
	SET( OPENNI_LIB /home/matthias/OpenNi_bin/Lib ) 
	
	SET( LIBLO_INCLUDES /home/matthias/liblo-0.26 ) 
	SET( LIBLO_LIB /usr/local/lib ) 

	#assuming freeglut installed on linux
	LINK_DIRECTORIES( ${OPENNI_LIB} ${OPENCV_LIB} ${LIBLO_LIB} )

ENDIF(WIN32)
	
INCLUDE_DIRECTORIES( . .. ${OPENNI_INCLUDES} ${OPENCV_INCLUDES} ${LIBLO_INCLUDES} )

	
add_executable( ../head_pose_estimation_demo ${HEAD_DEMO})


IF(WIN32)
target_link_libraries (../head_pose_estimation_demo opencv_core opencv_highgui opencv_imgproc OpenNI freeglut lo)
ELSE(WIN32)
target_link_libraries (../head_pose_estimation_demo opencv_core opencv_highgui opencv_imgproc OpenNI GLU glut lo)
ENDIF(WIN32)

