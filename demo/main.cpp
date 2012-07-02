/*
// Authors: Gabriele Fanelli, Thibaut Weise, Juergen Gall, BIWI, ETH Zurich
// Email: fanelli@vision.ee.ethz.ch


// When using this software, please acknowledge the effort that
// went into development by referencing the paper:
//
// Fanelli G., Weise T., Gall J., Van Gool L., Real Time Head Pose Estimation from Consumer Depth Cameras
// 33rd Annual Symposium of the German Association for Pattern Recognition (DAGM'11), 2011

*/

//#define USE_MS_SKD false

#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include "freeglut.h"

#ifndef USE_MS_SKD
	#include <XnOS.h>
	#include <XnCppWrapper.h>
	#include <XnCodecIDs.h>
	using namespace xn;
#else
	#include <d2d1.h>
	#include "NuiApi.h"
	//#include "DepthBasics.h"
#endif

#include "../CRForestEstimator.h"
#include "gl_camera.hpp"

// OSC added by Matthias Kronlachner
#include <lo/lo.h> // OSC
char *ADDRESS = "127.0.0.1";
char *PORT = "7120";
lo_address addr;
#define OUTPUT_BUFFER_SIZE 1024*16
char osc_buffer[OUTPUT_BUFFER_SIZE];
char tmp[50]; //Temp buffer for OSC address pattern

bool show_visual = TRUE;
bool send_osc = TRUE;


using namespace std;
using namespace cv;

// Path to trees
string g_treepath;
// Number of trees
int g_ntrees;
// Patch width
int g_p_width;
// Patch height
int g_p_height;
//maximum distance form the sensor - used to segment the person
int g_max_z = 0;
//head threshold - to classify a cluster of votes as a head
int g_th = 300;
//threshold for the probability of a patch to belong to a head
float g_prob_th = 1.0f;
//threshold on the variance of the leaves
float g_maxv = 400.f;
//stride (how densely to sample test patches - increase for higher speed)
int g_stride = 10;
//radius used for clustering votes into possible heads
float g_larger_radius_ratio = 1.f;
//radius used for mean shift
float g_smaller_radius_ratio = 5.f;
//
int g_frame_no = 0;
//opengl window size
int w,h;
//pointer to the actual estimator
CRForestEstimator* g_Estimate;
//input 3D image
Mat g_im3D, g_imD;
//input image size
int g_im_w = 640;
int g_im_h = 480;
//kinect's frame rate
int g_fps = 30;

#ifndef USE_MS_SKD
	XnUInt64 g_focal_length;
	XnDouble g_pixel_size;
	xn::Context g_Context;
	xn::DepthGenerator g_DepthGenerator;
	DepthMetaData g_depthMD;
	XnStatus g_RetVal;
#else
    INuiSensor*             g_pNuiSensor;
	HANDLE                  g_pDepthStreamHandle;
    HANDLE                  g_hNextDepthFrameEvent;
#endif

bool g_first_rigid = true;
bool g_show_votes = false;
bool g_draw_triangles = false;
bool g_draw = true;

//for interactive visualization
gl_camera g_camera;

std::vector< cv::Vec<float,POSE_SIZE> > g_means; //outputs
std::vector< std::vector< const Vote* > > g_clusters; //full clusters of votes
std::vector< Vote > g_votes; //all votes returned by the forest

math_vector_3f g_face_curr_dir, g_face_dir(0,0,-1);


void drawCylinder( const math_vector_3f& p1, const math_vector_3f& p2 , float radius, GLUquadric *quadric)
{
	math_vector_3f d = p2 - p1;
	if (d[2] == 0)
		d[2] = .0001f;

	float n = length(d);
	float ax = ( d[2] < 0.0 ) ? -57.295779f*acos( d[2]/n ) : 57.295779f*acos( d[2]/n );

	glPushMatrix();

	glTranslatef( p1[0],p1[1],p1[2] );
	glRotatef( ax, -d[1]*d[2], d[0]*d[2], 0.0);
	gluQuadricOrientation(quadric,GLU_OUTSIDE);
	gluCylinder(quadric, radius, radius, n, 10, 1);

	gluQuadricOrientation(quadric,GLU_INSIDE);
	gluDisk( quadric, 0.0, radius, 10, 1);
	glTranslatef( 0,0,n );

	gluQuadricOrientation(quadric,GLU_OUTSIDE);
	gluDisk( quadric, 0.0, radius, 10, 1);
	glPopMatrix();
}



bool initialize(){

	std::cout << "initializing kinect... " << endl;

#ifdef USE_MS_SKD

	int iSensorCount = 0;
    if ( NuiGetSensorCount(&iSensorCount) < 0 )
		return false;

    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i)
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        if ( NuiCreateSensorByIndex(i, &g_pNuiSensor) < 0 ) continue;
      
		// Get the status of the sensor, and if connected, then we can initialize it
        if( 0== g_pNuiSensor->NuiStatus() ){
            g_pNuiSensor = g_pNuiSensor;
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        g_pNuiSensor->Release();
    }

    if (NULL != g_pNuiSensor)
    {
        // Initialize the Kinect and specify that we'll be using depth
        if ( g_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH) >= 0 )
        {
            // Create an event that will be signaled when depth data is available
            g_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

            // Open a depth image stream to receive depth frames
            g_pNuiSensor->NuiImageStreamOpen(
                NUI_IMAGE_TYPE_DEPTH,
                NUI_IMAGE_RESOLUTION_640x480,
                0,
                2,
                g_hNextDepthFrameEvent,
                &g_pDepthStreamHandle);
        }
    }
	else
        return false;
	
#else

	// Initialize context object
	g_RetVal = g_Context.Init();

	g_RetVal = g_DepthGenerator.Create(g_Context);
	if (g_RetVal != XN_STATUS_OK)
		printf("Failed creating DEPTH generator %s\n", xnGetStatusString(g_RetVal));

	XnMapOutputMode outputMode;
	outputMode.nXRes = g_im_w;
	outputMode.nYRes = g_im_h;
	outputMode.nFPS = g_fps;
	g_RetVal = g_DepthGenerator.SetMapOutputMode(outputMode);
	if (g_RetVal != XN_STATUS_OK){
		printf("Failed setting the DEPTH output mode %s\n", xnGetStatusString(g_RetVal));
		return false;
	}

	g_RetVal = g_Context.StartGeneratingAll();
	if (g_RetVal != XN_STATUS_OK){
		printf("Failed starting generating all %s\n", xnGetStatusString(g_RetVal));
		return false;
	}

#endif

	g_im3D.create(g_im_h,g_im_w,CV_32FC3);
	g_imD.create(g_im_h,g_im_w,CV_16UC1);

	return true;
}

// load config file
void loadConfig(const char* filename) {

	ifstream in(filename);
	string dummy;

	if(in.is_open()) {

		// Path to trees
		in >> dummy;
		in >> g_treepath;

		// Number of trees
		in >> dummy;
		in >> g_ntrees;

		in >> dummy;
		in >> g_maxv;

		in >> dummy;
		in >> g_larger_radius_ratio;

		in >> dummy;
		in >> g_smaller_radius_ratio;

		in >> dummy;
		in >> g_stride;

		in >> dummy;
		in >> g_max_z;

		in >> dummy;
		in >> g_th;


	} else {
		cerr << "File not found " << filename << endl;
		exit(-1);
	}
	in.close();

	cout << endl << "------------------------------------" << endl << endl;
	cout << "Estimation:       " << endl;
	cout << "Trees:            " << g_ntrees << " " << g_treepath << endl;
	cout << "Stride:           " << g_stride << endl;
	cout << "Max Variance:     " << g_maxv << endl;
	cout << "Max Distance:     " << g_max_z << endl;
	cout << "Head Threshold:   " << g_th << endl;

	cout << endl << "------------------------------------" << endl << endl;

}


bool read_data( ){

#ifdef USE_MS_SKD

	const int eventCount = 1;
    void* hEvents[eventCount];
	hEvents[0] = g_hNextDepthFrameEvent;

	// Check to see if we have either a message (by passing in QS_ALLINPUT)
	// Or a Kinect event (hEvents)
	// Update() will check for Kinect events individually, in case more than one are signalled
	DWORD dwEvent = MsgWaitForMultipleObjects(eventCount, hEvents, FALSE, INFINITE, QS_ALLINPUT);

	// Check if this is an event we're waiting on and not a timeout or message
	if (WAIT_OBJECT_0 == dwEvent)
	{
		if (NULL == g_pNuiSensor)
			return false;

		if ( WAIT_OBJECT_0 == WaitForSingleObject(g_hNextDepthFrameEvent, 0) ) {

			NUI_IMAGE_FRAME imageFrame;
			HRESULT hr = g_pNuiSensor->NuiImageStreamGetNextFrame(	g_pDepthStreamHandle,
																	0,
																	&imageFrame );

			if ( FAILED( hr ) ) return false;
		
			INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
			NUI_LOCKED_RECT LockedRect;
			pTexture->LockRect( 0, &LockedRect, NULL, 0 );
			if ( 0 != LockedRect.Pitch )
			{
				g_imD.setTo(0);

				DWORD frameWidth, frameHeight;
				NuiImageResolutionToSize( imageFrame.eResolution, frameWidth, frameHeight );
        
				// draw the bits to the bitmap
				//BYTE * rgbrun = m_depthRGBX;
				const USHORT * pBufferRun = (const USHORT *)LockedRect.pBits;

				// end pixel is start + width*height - 1
				const USHORT * pBufferEnd = pBufferRun + (frameWidth * frameHeight);

				int pixel = 0;
				while ( pBufferRun < pBufferEnd )
				{
					USHORT depth     = *pBufferRun;
					USHORT realDepth = NuiDepthPixelToDepth(depth);
         
					int x = pixel%640;
					int y = floor(float(pixel)/640.f);
					
					g_imD.at<int16_t>(y,x) = realDepth;

					++pBufferRun;
					++pixel;
				}
			}
			
			// We're done with the texture so unlock it
			pTexture->UnlockRect(0);

			// Release the frame
			g_pNuiSensor->NuiImageStreamReleaseFrame(g_pDepthStreamHandle, &imageFrame);
			
		}
	}

#else

	// Wait for new data to be available
	g_RetVal = g_Context.WaitAndUpdateAll();
	if (g_RetVal != XN_STATUS_OK)
	{
		printf("Failed updating data: %s\n", xnGetStatusString(g_RetVal));
		return false;
	}

	// Take current depth map
	g_DepthGenerator.GetMetaData(g_depthMD);

#endif

	int valid_pixels = 0;
	float d = 0.f;

	//generate 3D image
	for(int y = 0; y < g_im3D.rows; y++)
	{
		Vec3f* Mi = g_im3D.ptr<Vec3f>(y);
		for(int x = 0; x < g_im3D.cols; x++){

#ifdef USE_MS_SKD
			d = float( g_imD.at<int16_t>(y,x) );
#else
			d = (float)g_depthMD(x,y);
#endif

			if ( d < g_max_z && d > 0 ){

				valid_pixels++;

				Mi[x][0] = ( float(d * (x - 320)) * 0.0017505f );
				Mi[x][1] = ( float(d * (y - 240)) * 0.0017505f );
				Mi[x][2] = d;

			}
			else
				Mi[x] = 0;

		}
	}

	//this part is to set the camera position, depending on what's in the scene
	if (g_first_rigid ) {

		if( valid_pixels > 10000){ //wait for something to be in the image

			// calculate gravity center
			Vec3f gravity(0,0,0);
			int count = 0;
			for(int y=0;y<g_im3D.rows;++y){
				const Vec3f* Mi = g_im3D.ptr<Vec3f>(y);
				for(int x=0;x<g_im3D.cols;++x){

					if( Mi[x][2] > 0 ) {

						gravity = gravity + Mi[x];
						count++;
					}
				}
			}

			float maxDist = 0;
			if(count > 0) {

				gravity = (1.f/(float)count)*gravity;

				for(int y=0;y<g_im3D.rows;++y){
					const Vec3f* Mi = g_im3D.ptr<Vec3f>(y);
					for(int x=0;x<g_im3D.cols;++x){

						if( Mi[x][2] > 0 ) {

							maxDist = MAX(maxDist,(float)norm( Mi[x]-gravity ));
						}
					}
				}
			}

			g_camera.resetview( math_vector_3f(gravity[0],gravity[1],gravity[2]), maxDist );
			g_camera.rotate_180();
			g_first_rigid = false;
		}
	}

	return true;
}

bool process() {

	if( read_data() ){

		g_means.clear();
		g_votes.clear();
		g_clusters.clear();

		//do the actual estimation
		g_Estimate->estimate( 	g_im3D,
									g_means,
									g_clusters,
									g_votes,
									g_stride,
									g_maxv,
									g_prob_th,
									g_larger_radius_ratio,
									g_smaller_radius_ratio,
									false,
									g_th
								);

		// OSC by Matthias Kronlachner
		
		if (send_osc)
		{
			for(unsigned int i=0;i<g_means.size();++i) {

				float x = g_means[i][0];
				float y = g_means[i][1];
				float z = g_means[i][2];		
				float pitch = g_means[i][3];
				float yaw = g_means[i][4];
				float roll = g_means[i][5];
			
				//cout << "user: " << i << "pitch: " << pitch << " yaw: " << yaw << " roll: " << roll << endl;
			
				lo_send(addr,"/head_pose", "fffffff", (float)i, x, y, z, pitch, yaw, roll);
			}
		}
		

		//if(g_means.size()>0)
		//	cout << g_means[0][0] << " " << g_means[0][1] << " " << g_means[0][2] << endl;
		return true;
	}
	else
		return false;

}

// ##############################################################################
void key(unsigned char _k, int, int) {

	switch(_k) {

		//case 'd': {

		//	g_Estimate->m_avg_votes = !g_Estimate->m_avg_votes;
		//	//g_draw = !g_draw;
		//	cout << "toggled avg votes " << g_draw << endl;
		//	break;

		//}

		case 's': {

			g_show_votes = !g_show_votes;
			cout << "toggled votes " << g_show_votes << endl;
			break;

		}

		case 't': {

			g_draw_triangles = !g_draw_triangles;
			cout << "toggled triangles " << g_draw_triangles << endl;
			break;

		}

		case '+': {

			g_stride++;
			cout << "stride : " << g_stride << endl;
			break;

		}
		case '-':{

			g_stride = MAX(0, g_stride-1);
			cout << "stride : " << g_stride << endl;
			break;

		}
		case '*': {

			g_th += 20;
			cout << "head threshold : " << g_th << endl;
			break;
		}

		case '/':{

			g_th -= 20;
			cout << "head threshold : " << g_th << endl;
			break;
		}

		case 'h':{

			printf("\nAvailable commands:\n");

			//printf("\t 'd' : toggle opengl display \n");

			printf("\t 's' : toggle votes display \n");

			printf("\t 't' : toggle triangles display \n");

			printf("\t '+' : increase stride \n");
			printf("\t '-' : decrease stride \n");

			printf("\t '*' : increase head threshold \n");
			printf("\t '/' : decrease head threshold \n");

			break;

		}

	default:
		break;

	}
	glutSwapBuffers();

}

// ##############################################################################
void resize(int _w, int _h) {
	w = _w;
	h = _h;
}

// ##############################################################################
void mm(int x, int y)
{
	y = h-y;
	g_camera.mouse_move(x,y);

}

// ##############################################################################
void mb(int button, int state, int x, int y)
{
	y = h-y;

	Mouse::button b = Mouse::NONE;

	if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {

		b = Mouse::ROTATE;
		g_camera.mouse(x,y,b);

	}
	else if(button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {

		b = Mouse::MOVEXY;
		g_camera.mouse(x,y,b);
	}
	else if((button & 3) == 3) {

		g_camera.mouse_wheel(20);
	}
	else if ((button & 4) == 4) {

		g_camera.mouse_wheel(-20);
	}


}

void idle(){

	process();
	g_frame_no++;

}

// draws the scan and the estimated head pose
void draw()
{
	if(1){

		glEnable(GL_NORMALIZE);
		glEnable(GL_DEPTH_TEST);

		g_camera.set_viewport(0,0,w,h);
		g_camera.setup();
		g_camera.use_light(true);

		glClearColor(1,1,1,1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDisable(GL_CULL_FACE);

		glPushMatrix();
		glColor3f(0.9f,0.9f,1.f);

		if(g_draw_triangles){

			math_vector_3f d1,d2;
			glBegin(GL_TRIANGLES);

			for(int y = 0; y < g_im3D.rows-1; y++)
			{

				const Vec3f* Mi = g_im3D.ptr<Vec3f>(y);
				const Vec3f* Mi1 = g_im3D.ptr<Vec3f>(y+1);

				for(int x = 0; x < g_im3D.cols-1; x++){

					if( Mi[x][2] <= 0 || Mi1[x][2] <= 0 || Mi[x+1][2] <= 0 || Mi1[x+1][2] <= 0 )
						continue;

					d1[0] = Mi[x][0] - Mi1[x][0];// v1 - v2;
					d1[1] = Mi[x][1] - Mi1[x][1];
					d1[2] = Mi[x][2] - Mi1[x][2];

					d2[0] = Mi[x+1][0] - Mi1[x][0];// v1 - v2;
					d2[1] = Mi[x+1][1] - Mi1[x][1];
					d2[2] = Mi[x+1][2] - Mi1[x][2];

					if ( fabs(d2[2])>20 || fabs(d1[2])>20 )
						continue;

					math_vector_3f norm = cross_product(d2,d1);

					glNormal3f(norm[0],norm[1],norm[2]);
					glVertex3f(Mi[x][0],Mi[x][1],Mi[x][2]);
					glVertex3f(Mi1[x][0],Mi1[x][1],Mi1[x][2]);
					glVertex3f(Mi[x+1][0],Mi[x+1][1],Mi[x+1][2]);
					glVertex3f(Mi1[x][0],Mi1[x][1],Mi1[x][2]);
					glVertex3f(Mi1[x+1][0],Mi1[x+1][1],Mi1[x+1][2]);
					glVertex3f(Mi[x+1][0],Mi[x+1][1],Mi[x+1][2]);

				}
			}
			glEnd();

		}
		else{

			math_vector_3f d1,d2;
			glBegin(GL_POINTS);

			for(int y = 0; y < g_im3D.rows-1; y++)
			{

				const Vec3f* Mi = g_im3D.ptr<Vec3f>(y);
				const Vec3f* Mi1 = g_im3D.ptr<Vec3f>(y+1);

				for(int x = 0; x < g_im3D.cols-1; x++){

					if( Mi[x][2] <= 0 || Mi[x][2] <= 0 )
						continue;

					d1[0] = Mi[x][0] - Mi1[x][0];// v1 - v2;
					d1[1] = Mi[x][1] - Mi1[x][1];
					d1[2] = Mi[x][2] - Mi1[x][2];

					d2[0] = Mi[x+1][0] - Mi1[x][0];// v1 - v2;
					d2[1] = Mi[x+1][1] - Mi1[x][1];
					d2[2] = Mi[x+1][2] - Mi1[x][2];

					math_vector_3f norm = cross_product(d2,d1);
					glNormal3f(norm[0],norm[1],norm[2]);
					glVertex3f(Mi[x][0],Mi[x][1],Mi[x][2]);

				}
			}
			glEnd();

		}

		glPopMatrix();

		GLUquadric* point = gluNewQuadric();
		GLUquadric *quadric = gluNewQuadric();
		gluQuadricNormals(quadric, GLU_SMOOTH);

		//draw head poses
		if(g_means.size()>0){

			glColor3f( 0, 1, 0);
			float mult = 0.0174532925f;

			for(unsigned int i=0;i<g_means.size();++i){

				rigid_motion<float> rm;
				rm.m_rotation = euler_to_rotation_matrix( mult*g_means[i][3], mult*g_means[i][4], mult*g_means[i][5] );
				math_vector_3f head_center( g_means[i][0], g_means[i][1], g_means[i][2] );

				glPushMatrix();
				glTranslatef( head_center[0], head_center[1], head_center[2] );
				gluSphere( point, 10.f, 10, 10 );
				glPopMatrix();

				g_face_curr_dir = rm.m_rotation * (g_face_dir);
				math_vector_3f head_front(head_center + 150.f*g_face_curr_dir);

				drawCylinder(head_center, head_front, 8, quadric);

			}

		}

		//draw the single votes
		if(g_show_votes){

			int rate = 1;
			glColor3f( 0 , 0, 1);

			for (unsigned int i = 0; i<g_votes.size();i+=rate){

				glPushMatrix();
				glTranslatef( g_votes[i].vote[0], g_votes[i].vote[1], g_votes[i].vote[2] );
				gluSphere( point, 2.f, 10, 10 );
				glPopMatrix();

			}

			for(unsigned int c=0;c<g_clusters.size();c++){

				switch(c%5){

					case 0 : glColor3f( 1.f, 0.f, 0.f); break;
					case 1 : glColor3f( 0.f, 1.f, 0.f); break;
					case 2 : glColor3f( 0.f, 0.f, 1.f); break;
					case 3 : glColor3f( 1.f, 0.f, 1.f); break;
					case 4 : glColor3f( 0.2f, 0.f, 0.8f); break;
					default : glColor3f( 0.f, 1.f, 1.f); break;

				}

				for(unsigned int i=0;i<g_clusters[c].size();i+=rate){

					glPushMatrix();
					glTranslatef(  g_clusters[c][i]->vote[0],  g_clusters[c][i]->vote[1],  g_clusters[c][i]->vote[2] );
					gluSphere( point, 3.f, 10, 10 );
					glPopMatrix();

				}

			}

		} //show votes

		gluDeleteQuadric(point);
		gluDeleteQuadric(quadric);

	}

	glutSwapBuffers();
	glutPostRedisplay();

}

int main(int argc, char* argv[])
{

	if( argc < 2 ){

		cout << "usage: ./head_demo config_file <show_visual send_osc osc_ip osc_port>" << endl;
		exit(-1);
	}

	loadConfig( argv[1] );
	g_Estimate =  new CRForestEstimator();
	if( !g_Estimate->load_forest(g_treepath.c_str(), g_ntrees) ){

		cerr << "could not read forest!" << endl;
		exit(-1);
	}

	if(!initialize()){
		cerr << "could not initialize Kinect!" << endl;
		exit(-1);
	}

	// visual on or off by MK
	if (argc > 2) {
		if (atoi(argv[2]) == 0)
		{ 
			show_visual = FALSE;
		} else {
			show_visual = TRUE;
		}
	}

	// osc on or off by MK
	if (argc > 3) {
		if (atoi(argv[3]) == 0)
		{ 
			send_osc = FALSE;
		} else {
			send_osc = TRUE;
		}
	}

	// OSC by Matthias Kronlachner
	if (send_osc)
	{
		if (argc > 4) {
			ADDRESS = argv[4];
		}
		if (argc > 5) {
			PORT = argv[5];
		}
		addr = lo_address_new(ADDRESS, PORT);
		printf("Configured to send OSC messages to %s:%s\n", ADDRESS, PORT);
	}

	

	if(show_visual){
		// initialize GLUT
		glutInitWindowSize(800, 600);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
		glutInit(&argc, argv);

		glutCreateWindow("HeadPoseDemo (press h for list of available commands)");
		glutDisplayFunc(draw);
		glutMouseFunc(mb);
		glutMotionFunc(mm);
		glutKeyboardFunc(key);
		glutReshapeFunc(resize);
		glutIdleFunc(idle);
		glutMainLoop();
	}
	else{

		while(1){

			process();

			
			if(g_means.size()>0){
			
				cout << g_means.size() << " " << flush;

				for(uint v=0;v<6;v++)
					cout << g_means[0][v] << " ";
				cout << endl;
			}
			g_frame_no++;


		}
	}

	return 0;


}
