#!/bin/sh
#
# changes path for libfreenect in pix_freenect.pd_darwin and libfreenect.0.0.1.dylib

install_name_tool -change /opt/local/lib/liblo.7.dylib @executable_path/mac-libs/liblo.7.dylib head_pose_estimation_demo
install_name_tool -change /Users/matthias/OpenCV-2.3.1/lib/libopencv_core.2.3.dylib @executable_path/mac-libs/libopencv_core.2.3.1.dylib head_pose_estimation_demo
install_name_tool -change /Users/matthias/OpenCV-2.3.1/lib/libopencv_imgproc.2.3.dylib @executable_path/mac-libs/libopencv_imgproc.2.3.1.dylib head_pose_estimation_demo
install_name_tool -change libfreenect.0.1.dylib @executable_path/mac-libs/libfreenect.0.1.2.dylib head_pose_estimation_demo


