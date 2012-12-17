/*-----------------------------------------------------------------
LOG
GEM - Graphics Environment for Multimedia

Clamp pixel values to a threshold

Copyright (c) 1997-1998 Mark Danks. mark@danks.org
Copyright (c) Günther Geiger. geiger@epy.co.at
Copyright (c) 2001-2011 IOhannes m zmölnig. forum::für::umläute. IEM. zmoelnig@iem.at
Copyright (c) 2002 James Tittle & Chris Clepper
For information on usage and redistribution, and for a DISCLAIMER OF ALL
WARRANTIES, see the file, "GEM.LICENSE.TERMS" in this distribution.

-----------------------------------------------------------------*/

#ifndef _INCLUDE__GEM_PIXES_pix_head_pose_estimation_H_
#define _INCLUDE__GEM_PIXES_pix_head_pose_estimation_H_

#include "Base/GemPixObj.h"
#include "Gem/Settings.h"

using namespace std;
using namespace cv;

/*-----------------------------------------------------------------
-------------------------------------------------------------------
CLASS
    pix_head_pose_estimation
    
    Clamp pixel values to a threshold

KEYWORDS
    pix
    
DESCRIPTION

    Inlet for a list - "vec_thresh"
    Inlet for a float - "ft1"
    
    "vec_thresh" - The threshold vector
    "ft1" - Set all thresholds to one value
   
-----------------------------------------------------------------*/
#ifdef _WIN32
class GEM_EXPORT pix_head_pose_estimation : public GemPixObj
#else
class GEM_EXTERN pix_head_pose_estimation : public GemPixObj
#endif
{
    CPPEXTERN_HEADER(pix_head_pose_estimation, GemPixObj);

    public:

        //////////
        // Constructor
    	pix_head_pose_estimation();
    	
			t_outlet 	*m_dataout;
			
			//////////      
			// settings
			float m_maxv;
			float m_larger_radius_ratio;
			float m_smaller_radius_ratio;
			int m_stride;
			float m_max_z;
			int m_th;
			float m_prob_th;
			string g_treepath;
			int g_ntrees;
			
    protected:
    	
    	//////////
    	// Destructor
    	virtual ~pix_head_pose_estimation();

    	//////////
    	// Do the processing
    	virtual void 	processRGBAImage(imageStruct &image);
    	virtual void 	processYUVImage(imageStruct &image);
    	
			
			//input image size
			int m_im_w;
			int m_im_h;
			
    private:
    
    	//////////
    	// Static member functions
    	static void 	floatMaxvMessCallback(void *data, t_floatarg arg);
			static void 	floatLargerRadiusRatioMessCallback(void *data, t_floatarg arg);
			static void 	floatSmallerRadiusRatioMessCallback(void *data, t_floatarg arg);
			static void 	floatStrideMessCallback(void *data, t_floatarg arg);
			static void 	floatMaxZMessCallback(void *data, t_floatarg arg);
			static void 	floatThMessCallback(void *data, t_floatarg arg);
			
};

#endif	// for header file
