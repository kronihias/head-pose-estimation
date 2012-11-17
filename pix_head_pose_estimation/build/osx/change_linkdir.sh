#!/bin/sh
#
# changes libpath for pix_head_pose_estimation.pd_darwin

install_name_tool -change @loader_path/libopencv_core.2.3.1.dylib  @loader_path/libopencv_core.2.3.dylib pix_head_pose_estimation.pd_darwin
install_name_tool -change @loader_path/libopencv_highgui.2.3.1.dylib  @loader_path/libopencv_highgui.2.3.dylib pix_head_pose_estimation.pd_darwin
install_name_tool -change @loader_path/libopencv_imgproc.2.3.1.dylib  @loader_path/libopencv_imgproc.2.3.dylib pix_head_pose_estimation.pd_darwin

