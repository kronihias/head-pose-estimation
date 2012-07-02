cmake_minimum_required(VERSION 2.6)

SET( 	HEAD_DEMO
	CRForestEstimator.cpp
	CRTree.cpp
	main.cpp 
)

SET(CMAKE_BUILD_TYPE "Release")

#modify according to your opencv installation
#SET( OPENCV_INCLUDES /include ) 
#SET( OPENCV_LIB /lib ) 

#SET( OPENCV_INCLUDES /usr/pack/opencv-2.3.1-cr/amd64-debian-linux6.0/include ) 	
#SET( OPENCV_LIB /usr/pack/opencv-2.3.1-cr/amd64-debian-linux6.0/lib ) 

SET( OPENCV_INCLUDES C:/Users/biwi/Kinect/libraries/opencv/include ) 
SET( OPENCV_LIB C:/Users/biwi/Kinect/libraries/opencv/lib ) 


INCLUDE_DIRECTORIES( . ${OPENCV_INCLUDES} )
LINK_DIRECTORIES( ${OPENCV_LIB} )

add_executable( head_pose_estimation ${HEAD_DEMO})

target_link_libraries (head_pose_estimation opencv_core241 opencv_highgui241 opencv_imgproc241 )




