/*
// Authors: Gabriele Fanelli, Thibaut Weise, Juergen Gall, BIWI, ETH Zurich
// Email: fanelli@vision.ee.ethz.ch

// You may use, copy, reproduce, and distribute this Software for any
// non-commercial purpose, subject to the restrictions of the
// Microsoft Research Shared Source license agreement ("MSR-SSLA").
// Some purposes which can be non-commercial are teaching, academic
// research, public demonstrations and personal experimentation. You
// may also distribute this Software with books or other teaching
// materials, or publish the Software on websites, that are intended
// to teach the use of the Software for academic or other
// non-commercial purposes.
// You may not use or distribute this Software or any derivative works
// in any form for commercial purposes. Examples of commercial
// purposes would be running business operations, licensing, leasing,
// or selling the Software, distributing the Software for use with
// commercial products, using the Software in the creation or use of
// commercial products or any other activity which purpose is to
// procure a commercial gain to you or others.
// If the Software includes source code or data, you may create
// derivative works of such portions of the Software and distribute
// the modified Software for non-commercial purposes, as provided
// herein.

// THE SOFTWARE COMES "AS IS", WITH NO WARRANTIES. THIS MEANS NO
// EXPRESS, IMPLIED OR STATUTORY WARRANTY, INCLUDING WITHOUT
// LIMITATION, WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A
// PARTICULAR PURPOSE, ANY WARRANTY AGAINST INTERFERENCE WITH YOUR
// ENJOYMENT OF THE SOFTWARE OR ANY WARRANTY OF TITLE OR
// NON-INFRINGEMENT. THERE IS NO WARRANTY THAT THIS SOFTWARE WILL
// FULFILL ANY OF YOUR PARTICULAR PURPOSES OR NEEDS. ALSO, YOU MUST
// PASS THIS DISCLAIMER ON WHENEVER YOU DISTRIBUTE THE SOFTWARE OR
// DERIVATIVE WORKS.

// NEITHER MICROSOFT NOR ANY CONTRIBUTOR TO THE SOFTWARE WILL BE
// LIABLE FOR ANY DAMAGES RELATED TO THE SOFTWARE OR THIS MSR-SSLA,
// INCLUDING DIRECT, INDIRECT, SPECIAL, CONSEQUENTIAL OR INCIDENTAL
// DAMAGES, TO THE MAXIMUM EXTENT THE LAW PERMITS, NO MATTER WHAT
// LEGAL THEORY IT IS BASED ON. ALSO, YOU MUST PASS THIS LIMITATION OF
// LIABILITY ON WHENEVER YOU DISTRIBUTE THE SOFTWARE OR DERIVATIVE
// WORKS.

// When using this software, please acknowledge the effort that
// went into development by referencing the paper:
//
// Fanelli G., Weise T., Gall J., Van Gool L., Real Time Head Pose Estimation from Consumer Depth Cameras
// 33rd Annual Symposium of the German Association for Pattern Recognition (DAGM'11), 2011

*/

#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include <stdint.h>
#include "CRForestEstimator.h"

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
int g_stride = 5;
//radius used for clustering votes into possible heads
float g_larger_radius_ratio = 1.f;
//radius used for mean shift
float g_smaller_radius_ratio = 6.f;

//pointer to the actual estimator
CRForestEstimator* g_Estimate;
//input 3D image
Mat g_im3D;

std::vector< cv::Vec<float,POSE_SIZE> > g_means; //outputs
std::vector< std::vector< const Vote* > > g_clusters; //full clusters of votes
std::vector< Vote > g_votes; //all votes returned by the forest

bool loadDepthImageCompressed(Mat& depthImg, const char* fname ){

	//now read the depth image
	FILE* pFile = fopen(fname, "rb");
	if(!pFile){
		cerr << "could not open file " << fname << endl;
		return false;
	}

	int im_width = 0;
	int im_height = 0;
	bool success = true;

	success &= ( fread(&im_width,sizeof(int),1,pFile) == 1 ); // read width of depthmap
	success &= ( fread(&im_height,sizeof(int),1,pFile) == 1 ); // read height of depthmap

	depthImg.create( im_height, im_width, CV_16SC1 );
	depthImg.setTo(0);


	int numempty;
	int numfull;
	int p = 0;

	if(!depthImg.isContinuous())
	{
		cerr << "Image has the wrong size! (should be 640x480)" << endl;
		return false;
	}

	int16_t* data = depthImg.ptr<int16_t>(0);
	while(p < im_width*im_height ){

		success &= ( fread( &numempty,sizeof(int),1,pFile) == 1 );

		for(int i = 0; i < numempty; i++)
			data[ p + i ] = 0;

		success &= ( fread( &numfull,sizeof(int), 1, pFile) == 1 );
		success &= ( fread( &data[ p + numempty ], sizeof(int16_t), numfull, pFile) == (unsigned int) numfull );
		p += numempty+numfull;

	}

	fclose(pFile);

	return success;
}

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

int main(int argc, char* argv[])
{

	if( argc != 3 ){

		cout << "usage: ./head_pose_estimation config_file depth_image" << endl;
		exit(-1);
	}

	loadConfig(argv[1]);
	CRForestEstimator estimator;
	if( !estimator.load_forest(g_treepath.c_str(), g_ntrees) ){

		cerr << "could not read forest!" << endl;
		exit(-1);
	}

	string depth_fname(argv[2]);

	//read calibration file (should be in the same directory as the depth image!)
	string cal_filename = depth_fname.substr(0,depth_fname.find_last_of("/")+1);

	cal_filename += "depth.cal";
	ifstream is(cal_filename.c_str());
	if (!is){
		cerr << "depth.cal file not found in the same folder as the depth image! " << endl;
		return -1;
	}
	//read intrinsics only
	float depth_intrinsic[9];
	for(int i =0; i<9; ++i)	is >> depth_intrinsic[i];
	is.close();

	Mat depthImg;
	//read depth image (compressed!)
	if (!loadDepthImageCompressed( depthImg, depth_fname.c_str() ))
		return -1;

	Mat img3D;
	img3D.create( depthImg.rows, depthImg.cols, CV_32FC3 );

	//get 3D from depth
	for(int y = 0; y < img3D.rows; y++)
	{
		Vec3f* img3Di = img3D.ptr<Vec3f>(y);
		const int16_t* depthImgi = depthImg.ptr<int16_t>(y);

		for(int x = 0; x < img3D.cols; x++){

			float d = (float)depthImgi[x];

			if ( d < g_max_z && d > 0 ){

				img3Di[x][0] = d * (float(x) - depth_intrinsic[2])/depth_intrinsic[0];
				img3Di[x][1] = d * (float(y) - depth_intrinsic[5])/depth_intrinsic[4];
				img3Di[x][2] = d;

			}
			else{

				img3Di[x] = 0;
			}

		}
	}

	g_means.clear();
	g_votes.clear();
	g_clusters.clear();

	string pose_filename(depth_fname.substr(0,depth_fname.find_last_of('_')));
	pose_filename += "_pose.bin";

	cv::Vec<float,POSE_SIZE> gt;
	bool have_gt = false;
	//try to read in the ground truth from a binary file
	FILE* pFile = fopen(pose_filename.c_str(), "rb");
	if(pFile){

		have_gt = true;
		have_gt &= ( fread( &gt[0], sizeof(float),POSE_SIZE, pFile) == POSE_SIZE );
		fclose(pFile);

	}

	//do the actual estimate
	estimator.estimate_new( (const cv::Mat&)img3D,
								 g_means,
								 g_clusters,
								 g_votes,
								 g_stride,
								 g_maxv,
								 g_prob_th,
								 g_larger_radius_ratio,
								 g_smaller_radius_ratio,
								 false,
								 g_th );

	cout << "Heads found : " << g_means.size() << endl;

	//assuming there's only one head in the image!
	if(g_means.size()>0){

		cout << "Estimated: " << g_means[0][0] << " " << g_means[0][1] << " " << g_means[0][2] << " " << g_means[0][3] << " " << g_means[0][4] << " " << g_means[0][5] <<endl;

		float pt2d_est[2];
		float pt2d_gt[2];

		if(have_gt){
			cout << "Ground T.: " << gt[0] << " " << gt[1] << " " << gt[2] << " " << gt[3] << " " << gt[4] << " " << gt[5] <<endl;

			cv::Vec<float,POSE_SIZE> err = (gt-g_means[0]);
			//multiply(err,err,err);
			for(int n=0;n<POSE_SIZE;++n)
				err[n] = err[n]*err[n];

			float h_err = sqrt(err[0]+err[1]+err[2]);
			float a_err = sqrt(err[3]+err[4]+err[5]);

			cout << "Head error : " << h_err << " mm " << endl;
			cout << "Angle error : " << a_err <<" degrees " <<  endl;

			pt2d_gt[0] = depth_intrinsic[0]*gt[0]/gt[2] + depth_intrinsic[2];
			pt2d_gt[1] = depth_intrinsic[4]*gt[1]/gt[2] + depth_intrinsic[5];

		}

		pt2d_est[0] = depth_intrinsic[0]*g_means[0][0]/g_means[0][2] + depth_intrinsic[2];
		pt2d_est[1] = depth_intrinsic[4]*g_means[0][1]/g_means[0][2] + depth_intrinsic[5];

	}

	return 0;

}

