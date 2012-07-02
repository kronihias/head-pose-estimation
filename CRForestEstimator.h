#pragma once

#ifdef WIN32
#include <stdint.h>
#endif

#include "CRTree.h"
#include <fstream>
#include "opencv2/core/core.hpp"


struct Vote {

	cv::Vec<float,POSE_SIZE> vote;
	const float* trace;
	const float* conf;
	//bool operator<(const Vote& a) const { return *trace < *(a.trace); }

};

struct VoteIndex {

	int index;
	const float* trace;
	bool operator<(const VoteIndex& a) const { return *trace < *(a.trace); }

};

class CRForestEstimator {

public:

	CRForestEstimator(){ m_avg_votes = true; };
	~CRForestEstimator();

	bool load_forest(const std::string& path, unsigned int ntrees = 0  );

	void do_regression(const cv::Mat & im3D, int stride, float max_variance,  float prob_th, std::vector< Vote >& votes );

	void do_clustering(const std::vector< Vote >& votes,
						float larger_radius_ratio,
						unsigned int max_clusters,
						cv::Vec<float,POSE_SIZE>& temp_mean,
						std::vector< std::vector< const Vote* > >& temp_clusters,
						std::vector< cv::Vec<float,POSE_SIZE> >& cluster_means,
						std::vector< std::vector< VoteIndex > >& cluster_votes_indeces);


	void estimate( const cv::Mat & im3D, //input: 3d image (x,y,z coordinates for each pixel)
					   std::vector< cv::Vec<float,POSE_SIZE> >& means, //output: heads' centers and orientations (x,y,z,pitch,yaw,roll)
					   std::vector< std::vector< const Vote* > >& clusters, //all clusters
					   std::vector< Vote >& votes, //all votes
					   int stride = 5, //stride
					   float max_variance = 1000, //max leaf variance
					   float prob_th = 1.0, //threshold on the leaf's probability of belonging to a head
					   float larger_radius_ratio = 1.0, //for clustering heads
					   float smaller_radius_ratio = 6.0, //for mean shift
					   bool verbose = false, //print out more info
					   int threshold = 400, //head threshold
					   int max_no_faces = 2);

	bool m_avg_votes;

private:

	
	std::vector< CRTree* > crForest;

};


