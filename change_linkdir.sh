#!/bin/sh
#
# changes libpath for pix_head_pose_estimation.pd_darwin

install_name_tool -change @executable_path/mac-libs/libopencv_core.2.3.1.dylib  @executable_path/mac-libs/libopencv_core.2.3.dylib head_pose_estimation_demo_freenect
install_name_tool -change @executable_path/mac-libs/libopencv_highgui.2.3.1.dylib  @executable_path/mac-libs/libopencv_highgui.2.3.dylib head_pose_estimation_demo_freenect
install_name_tool -change @executable_path/mac-libs/libopencv_imgproc.2.3.1.dylib  @executable_path/mac-libs/libopencv_imgproc.2.3.dylib head_pose_estimation_demo_freenect

