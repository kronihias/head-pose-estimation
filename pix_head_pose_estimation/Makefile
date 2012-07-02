# change to your local directories!
PD_APP_DIR = /Applications/Pd-extended.app/Contents/Resources
GEM_DIR = /Users/matthias/Gem-0.93.3
OPENCV_DIR = /Users/matthias/OpenCV-2.3.1

CPPFLAGS = -I$(GEM_DIR)/src -I$(PD_APP_DIR)/include

#linux doesnt work yet
UNAME := $(shell uname -s)
ifeq ($(UNAME),Linux)
 CPPFLAGS += -I/usr/include/ni
 CXXFLAGS = -g -O3 -fPIC -freg-struct-return -Os -falign-loops=32 -falign-functions=32 -falign-jumps=32 -funroll-loops -ffast-math -mmmx
 LDFLAGS = -shared -rdynamic
 LIBS = 
 EXTENSION = pd_linux
 USER_EXTERNALS=$(HOME)/pd-externals
endif
ifeq ($(UNAME),Darwin)
 CPPFLAGS += -I$(OPENCV_DIR)/modules/core/include -I$(OPENCV_DIR)/modules/highgui/include -I$(OPENCV_DIR)/modules/imgproc/include
 CXXFLAGS = -arch i386 -g -O3 -fast -msse3
 LDFLAGS = -arch i386 -bundle -bundle_loader $(PD_APP_DIR)/bin/pd -undefined dynamic_lookup -mmacosx-version-min=10.6 -L$(OPENCV_DIR)/lib
 LIBS = -lm -lopencv_core -lopencv_highgui -lopencv_imgproc
 EXTENSION = pd_darwin
 USER_EXTERNALS=$(HOME)/Library/Pd
endif

.SUFFIXES = $(EXTENSION)

SOURCES = pix_head_pose_estimation

all:
	g++ $(CPPFLAGS) $(CXXFLAGS) -o pix_head_pose_estimation.o -c pix_head_pose_estimation.cpp
	g++ $(CPPFLAGS) $(CXXFLAGS) -o ../CRForestEstimator.o -c  ../CRForestEstimator.cpp
	g++ $(CPPFLAGS) $(CXXFLAGS) -o ../CRTree.o -c  ../CRTree.cpp
	#g++ $(CPPFLAGS) $(CXXFLAGS) -o $(SOURCES).o -c $(SOURCES).cpp
	g++ -o $(SOURCES).$(EXTENSION) $(LDFLAGS) pix_head_pose_estimation.o ../CRForestEstimator.o ../CRTree.o $(LIBS)
	rm -fr ./*.o

clean:
	rm -f $(SOURCES)*.o
	rm -f $(SOURCES)*.$(EXTENSION)

distro: clean all
	rm *.o
