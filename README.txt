
kinect head pose estimation with OSC support

Includes standalone version and pix_head_pose_estimation Pure Data/Gem external.

Standaloe works with libfreenect, OpenNI or Microsoft Kinect SDK (Windows only)

based on 
Real Time Head Pose Estimation from Consumer Depth Cameras 
by Gabriele Fanelli
http://www.vision.ee.ethz.ch/~gfanelli/head_pose/head_forest.html

======================================

Application that detects head position from a depth image provided by
Kinect Sensor in x,y,z and Euler Angles (pitch, yaw, roll) 
from multiple persons.

Application sends data as OSC Message in the format:

/head_pose [User_ID] [x] [y] [z] [pitch] [yaw] [roll]

all arguments are float, angles in degree, User_ID starting at zero.

Usage:
* #.../head_pose_estimation> ./head_pose_estimation_demo config.txt <show visual 0 or 1> <send osc 0 or 1> <osc-ip> <osc-port>

example how to not show visualization and use custom ip and port for sending OSC Messages:

./head_pose_estimation_demo config.txt 0 1 192.168.0.1 8000

Default IP/Port: 127.0.0.1:7120


* you can find an example puredata/GEM patch in the folder pd	
to visualize the headtracking.

(C) 2011/2012 by Matthias Kronlachner
__________________________________________________________

::INSTALL STANDALONE PROGRAM::
Ready to use Binaries for OSX (64 bit) and Windows (32 bit) are included.


Windows
-----------------
Binary is included
Windows version is compiled for Microsoft Kinect SDK!!

* get and install OpenCV from http://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.0/
* get and install Microsoft Kinect SDK for Windows http://www.microsoft.com/en-us/kinectforwindows/develop/developer-downloads.aspx


Linux
-----------------

*	you need cmake, OpenCV, OpenNI, freeglut, and OpenGL libraries
	for installing OpenNI follow this instructions: https://github.com/avin2/SensorKinect

*	get liblo
	http://liblo.sourceforge.net/

*	edit demo/CMakeLists.txt and adjust paths

*	#.../head_pose_estimation/demo> cmake CMakeLists.txt
*	#.../head_pose_estimation/demo> make


*	#.../head_pose_estimation> ./head_pose_estimation_demo config.txt

OSX
-----------------

-> use precompiled BINARIES! 
	./head_pose_estimation_demo_freenect
or	./head_pose_estimation_demo_openni

for compilation:

* you will need XCode to compile the demo application for OSX 

* follow instructions on https://github.com/avin2/SensorKinect 
	to install OpenNI
	 
* get liblo
	http://liblo.sourceforge.net/

* get OpenCV, OpenGL, Glut through macports

* open the XCode Project in demo folder, 
  adjust path settings for Frameworks and build it.

__________________
:::THINGS TO DO:::




questions: m.kronlachner@gmail.com


