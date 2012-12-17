////////////////////////////////////////////////////////
//
// GEM - Graphics Environment for Multimedia
//
// zmoelnig@iem.kug.ac.at
//
// Implementation file
//
//    Copyright (c) 1997-1998 Mark Danks.
//    Copyright (c) Günther Geiger.
//    Copyright (c) 2001-2011 IOhannes m zmölnig. forum::für::umläute. IEM. zmoelnig@iem.at
//    For information on usage and redistribution, and for a DISCLAIMER OF ALL
//    WARRANTIES, see the file, "GEM.LICENSE.TERMS" in this distribution.
//
/////////////////////////////////////////////////////////


//#include <string>
#include <algorithm>
//#include <iostream>
//#include <vector>
#include "Gem/Exception.h"
//#include "m_pd.h"
#include "../CRForestEstimator.h"

#include "pix_head_pose_estimation.h"

using namespace std;
using namespace cv;



CPPEXTERN_NEW(pix_head_pose_estimation);

bool g_first_rigid = true;

//pointer to the actual estimator

CRForestEstimator* g_Estimate;

//input 3D image
Mat g_im3D;

// STORAGE
std::vector< cv::Vec<float,POSE_SIZE> > g_means; //outputs
std::vector< std::vector< const Vote* > > g_clusters; //full clusters of votes
std::vector< Vote > g_votes; //all votes returned by the forest


// used for finding external path
// taken from mrpeach/which
#ifdef __FreeBSD__
static char sys_dllextent[] = ".b_i386", sys_dllextent2[] = ".pd_freebsd";
#endif
#ifdef __linux__
#ifdef __x86_64__
static char sys_dllextent[] = ".l_ia64", sys_dllextent2[] = ".pd_linux";
#else
static char sys_dllextent[] = ".l_i386", sys_dllextent2[] = ".pd_linux";
#endif
#endif
#ifdef __APPLE__
#ifndef MACOSX3
static char sys_dllextent[] = ".d_fat", sys_dllextent2[] = ".pd_darwin";
#else
static char sys_dllextent[] = ".d_ppc", sys_dllextent2[] = ".pd_darwin";
#endif
#endif
#ifdef _WIN32
static char sys_dllextent[] = ".m_i386", sys_dllextent2[] = ".dll";
#endif

/////////////////////////////////////////////////////////
//
// pix_head_pose_estimation
//
/////////////////////////////////////////////////////////
// Constructor
//
/////////////////////////////////////////////////////////
pix_head_pose_estimation :: pix_head_pose_estimation()
{
		m_dataout = outlet_new(this->x_obj, 0);
		
		post("pix_head_pose_estimation - experimental - 2012 by Matthias Kronlachner");

		// TREES
		
		g_ntrees = 10;
		g_treepath = "./trees/new_";
		m_maxv = 500.0;
		m_larger_radius_ratio = 1.4;
		m_smaller_radius_ratio = 5.0;
		m_stride = 6;
		m_max_z = 1300.0;
		m_th = 400;
		
		float m_prob_th = 1.0f;
		
		m_im_w = 640;
		m_im_h = 480;

    // get external path for loading tree
    // taken from mrpeach/which
    int     fd = -1, result = 0;
    char    *nameptr = 0;
    char dirbuf[MAXPDSTRING];
    
    fd = canvas_open(canvas_getcurrent(), "pix_head_pose_estimation", sys_dllextent,
                     dirbuf, &nameptr, MAXPDSTRING, 1);
    if (fd < 0)
    {/* same, with the more generic sys_dllextent2 */
        fd = canvas_open(canvas_getcurrent(), "pix_head_pose_estimation", sys_dllextent2,
                         dirbuf, &nameptr, MAXPDSTRING, 1);
    }
    if (fd < 0)
    {
        throw(GemException("error reading folder! \n"));
    }
    
    result = close(fd);
    
    strcat(dirbuf, "/trees/new_");
    post("reading trees from: %s",dirbuf);
    
    
        g_Estimate =  new CRForestEstimator();
		if( !g_Estimate->load_forest(dirbuf, g_ntrees) ){
            throw(GemException("could not read forest! \n"));
		}
		
    
		// creade 3d images
		g_im3D.create(m_im_h,m_im_w,CV_32FC3);
    
    post ("created 3d image");
		
}

/////////////////////////////////////////////////////////
// Destructor
//
/////////////////////////////////////////////////////////
pix_head_pose_estimation :: ~pix_head_pose_estimation()
{ }

/////////////////////////////////////////////////////////
// processImage
//
/////////////////////////////////////////////////////////
void pix_head_pose_estimation :: processRGBAImage(imageStruct &image)
{
  // Read Data
	int datasize = image.xsize * image.ysize;

    unsigned char *base = image.data;
		int value = 0;
		
		int valid_pixels = 0;
		float d = 0.f;
		
			for(int y = 0; y < g_im3D.rows; y++)
			{
				Vec3f* Mi = g_im3D.ptr<Vec3f>(y);
				for(int x = 0; x < g_im3D.cols; x++){

					//d = (float)g_depthMD(x,y);
					
					d = (float)((int)base[chRed] << 8) + (int)base[chGreen];

					if ( d < m_max_z && d > 0 ){

						valid_pixels++;

						Mi[x][0] = ( float(d * (x - 320)) * 0.0017505f );
						Mi[x][1] = ( float(d * (y - 240)) * 0.0017505f );
						Mi[x][2] = d;

					}
					else
						Mi[x] = 0;

					base += 4;
				}
			}
		
		// reset storage
		g_means.clear();
		g_votes.clear();
		g_clusters.clear();

    
		//do the actual estimation
		g_Estimate->estimate(g_im3D,
									g_means,
									g_clusters,
									g_votes,
									m_stride,
									m_maxv,
									m_prob_th,
									m_larger_radius_ratio,
									m_smaller_radius_ratio,
									false,
									m_th
								);

            // Output Data
			for(unsigned int i=0;i<g_means.size();++i) {
				
				t_atom ap[7];
				SETFLOAT (ap+0, i); // id
				SETFLOAT (ap+1, g_means[i][0]); // x
				SETFLOAT (ap+2, g_means[i][1]); // y
				SETFLOAT (ap+3, g_means[i][2]); // z
				SETFLOAT (ap+4, g_means[i][3]); // pitch
				SETFLOAT (ap+5, g_means[i][4]); // yaw
				SETFLOAT (ap+6, g_means[i][5]); // roll
				
				outlet_anything(m_dataout, gensym("head_pose"), 7, ap);
			}

}

/////////////////////////////////////////////////////////
// YUV Depth Image --> Has to be corrected
//
/////////////////////////////////////////////////////////
void pix_head_pose_estimation :: processYUVImage(imageStruct &image)
{
    int datasize = image.xsize * image.ysize;

    unsigned char *base = image.data;
		int value = 0;
    while(datasize--)
    {

		base += 2;
    }    
}

/////////////////////////////////////////////////////////
// static member function
//
/////////////////////////////////////////////////////////
void pix_head_pose_estimation :: obj_setupCallback(t_class *classPtr)
{
    class_addmethod(classPtr, reinterpret_cast<t_method>(&pix_head_pose_estimation::floatMaxvMessCallback),
    	    gensym("maxv"), A_FLOAT, A_NULL);
    class_addmethod(classPtr, reinterpret_cast<t_method>(&pix_head_pose_estimation::floatLargerRadiusRatioMessCallback),
    	    gensym("larger_radius_ratio"), A_FLOAT, A_NULL);
    class_addmethod(classPtr, reinterpret_cast<t_method>(&pix_head_pose_estimation::floatSmallerRadiusRatioMessCallback),
    	    gensym("smaller_radius_ratio"), A_FLOAT, A_NULL);
    class_addmethod(classPtr, reinterpret_cast<t_method>(&pix_head_pose_estimation::floatStrideMessCallback),
    	    gensym("stride"), A_FLOAT, A_NULL);
    class_addmethod(classPtr, reinterpret_cast<t_method>(&pix_head_pose_estimation::floatMaxZMessCallback),
    	    gensym("max_z"), A_FLOAT, A_NULL);
    class_addmethod(classPtr, reinterpret_cast<t_method>(&pix_head_pose_estimation::floatThMessCallback),
    	    gensym("th"), A_FLOAT, A_NULL);
}
void pix_head_pose_estimation :: floatMaxvMessCallback(void *data, t_floatarg arg)
{
	pix_head_pose_estimation *me = (pix_head_pose_estimation*)GetMyClass(data);
  me->m_maxv=(float)arg;
}
void pix_head_pose_estimation :: floatLargerRadiusRatioMessCallback(void *data, t_floatarg arg)
{
	pix_head_pose_estimation *me = (pix_head_pose_estimation*)GetMyClass(data);
  me->m_larger_radius_ratio=(float)arg;
}
void pix_head_pose_estimation :: floatSmallerRadiusRatioMessCallback(void *data, t_floatarg arg)
{
	pix_head_pose_estimation *me = (pix_head_pose_estimation*)GetMyClass(data);
  me->m_smaller_radius_ratio=(float)arg;
}
void pix_head_pose_estimation :: floatStrideMessCallback(void *data, t_floatarg arg)
{
	pix_head_pose_estimation *me = (pix_head_pose_estimation*)GetMyClass(data);
  me->m_stride=(float)arg;
}
void pix_head_pose_estimation :: floatMaxZMessCallback(void *data, t_floatarg arg)
{
	pix_head_pose_estimation *me = (pix_head_pose_estimation*)GetMyClass(data);
  me->m_max_z=(float)arg;
}
void pix_head_pose_estimation :: floatThMessCallback(void *data, t_floatarg arg)
{
	pix_head_pose_estimation *me = (pix_head_pose_estimation*)GetMyClass(data);
  me->m_th=(float)arg;
}
