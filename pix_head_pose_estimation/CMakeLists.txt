cmake_minimum_required (VERSION 2.6) 

SET( 	head_pose_estimation
		../CRForestEstimator.cpp
		../CRTree.cpp
		pix_head_pose_estimation.cpp
)

SET(CMAKE_BUILD_TYPE "Release")

SET( PD_INC C:/Users/GVITC/Documents/_Matthias/pd-bin/src ) 
SET( PD_LIB C:/Users/GVITC/Documents/_Matthias/pd-bin/bin ) 

SET( GEM_INC C:/Users/GVITC/Documents/_Matthias/Gem-0.93.3-bin/Gem/dev ) 
SET( GEM_LIB C:/Users/GVITC/Documents/_Matthias/Gem-0.93.3-bin/Gem/dev ) 

SET( OPENCV_INC C:/Users/GVITC/Downloads/opencv/opencv/include/opencv ) 
SET( OPENCV_LIB C:/Users/GVITC/Downloads/opencv/opencv/build/x86/vc10/lib ) 

INCLUDE_DIRECTORIES( . .. ${PD_INC} ${GEM_INC} ${OPENCV_INC} )
LINK_DIRECTORIES( ${PD_LIB} ${GEM_LIB} ${OPENCV_LIB} )

add_library (pix_head_pose_estimation SHARED ${head_pose_estimation}) 

target_link_libraries (pix_head_pose_estimation Gem pd opencv_core242 opencv_highgui242 opencv_imgproc242)