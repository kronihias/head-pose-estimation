

#include "CRForestEstimator.h"
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
//using namespace img_utils;

struct ClusterIndex {
	float val;
	unsigned int index;
	bool operator<(const ClusterIndex& a) const { return val<a.val; }
};

cv::Rect getBoundingBox(const cv::Mat& im3D, int p_w, int p_h){

	cv::Rect bbox;
	int min_x = im3D.cols;
	int min_y = im3D.rows;
	int max_x = 0;
	int max_y = 0;

	for(int y = 0; y < im3D.rows; y++){
		for(int x = 0; x < im3D.cols; x++){
			if( im3D.at<cv::Vec3f>(y,x)[2] > 0.f ) {
				min_x = MIN(min_x,x);
				min_y = MIN(min_y,y);
				max_x = MAX(max_x,x);
				max_y = MAX(max_y,y);
			}
		}
	}

	int new_w = max_x - min_x + p_w;
	int new_h = max_y - min_y + p_h;

	bbox.x = MIN( im3D.cols-1, MAX( 0 , min_x - p_w/2 ) );
	bbox.y = MIN( im3D.rows-1, MAX( 0 , min_y - p_h/2) );

	bbox.width  = MAX(0, MIN( new_w, im3D.cols-bbox.x));
	bbox.height = MAX(0, MIN( new_h, im3D.rows-bbox.y));

	return bbox;

}

CRForestEstimator::~CRForestEstimator() {

	for(unsigned int i=0;i<crForest.size();++i)
		if (crForest[i])
			delete crForest[i];

}


bool CRForestEstimator::load_forest( const std::string& path,
									unsigned int no_trees ){


	crForest.resize(no_trees);

	for(unsigned int t=0;t<no_trees;++t){

		char buffer[200];
		sprintf(buffer,"%s%03d.tree", path.c_str(), t);
		std::string tree_name = buffer;

		CRTree* tree = new CRTree();

		if(!tree->read_tree( tree_name ))
			return false;

		crForest[t] = tree;
		printf ("loaded %s\n", buffer);

	}

	return true;
}



void CRForestEstimator::do_regression( const Mat & im3D, int stride, float max_variance, float prob_th, std::vector< Vote >& votes ){

	int p_width = int(crForest[0]->m_p_w);
	int p_height = int(crForest[0]->m_p_h);

	//get bounding box around the data in the image
	Rect bbox = getBoundingBox(im3D,p_width,p_height);

	Mat* channels = new Mat[3];
	split(im3D, channels);

	int rows = im3D.rows;
	int cols = im3D.cols;

	Mat_<float> depth = channels[2].clone();

	depth *= SCALE;

	//integral image of the depth channel
	Mat depthInt( rows+1, cols+1, CV_64FC1);
	integral( depth, depthInt );

	//feature channels vector, in our case it contains only the depth integral image, but it could have other channels (e.g., greyscale)
	std::vector< cv::Mat > featureChans;
	featureChans.push_back( depthInt );

	//mask
	Mat_<float> mask( rows, cols); mask.setTo(0);
	cv::threshold( depth, mask, 10, 1, THRESH_BINARY);

	//integral image of the mask
	Mat maskIntegral( rows+1, cols+1, CV_64FC1);
	maskIntegral.setTo(0);
	integral( mask, maskIntegral );

	//defines the test patch
	Rect roi = Rect(0,0,p_width,p_height);

	int min_no_pixels = p_width*p_height/10;
	int half_w = roi.width/2;
	int half_h = roi.height/2;

	//process each patch
	for(roi.y=bbox.y; roi.y<bbox.y+bbox.height-p_height; roi.y+=stride) {

		float* rowX = channels[0].ptr<float>(roi.y + half_h);
		float* rowY = channels[1].ptr<float>(roi.y + half_h);
		float* rowZ = channels[2].ptr<float>(roi.y + half_h);

		double* maskIntY1 = maskIntegral.ptr<double>(roi.y);
		double* maskIntY2 = maskIntegral.ptr<double>(roi.y + roi.height);

		for(roi.x=bbox.x; roi.x<bbox.x+bbox.width-p_width; roi.x+=stride) {

			//discard if the middle of the patch does not have depth data
			if( rowZ[roi.x + half_w] <= 0.f )
				continue;

			//discard if the patch is filled with data for less than 10%
			if( (maskIntY1[roi.x] + maskIntY2[roi.x + roi.width] - maskIntY1[roi.x + roi.width] - maskIntY2[roi.x]) <= min_no_pixels )
			   continue;

			//send the patch down the trees and retrieve leaves
			std::vector< const leaf_data* > leaves( crForest.size() );

			for(unsigned int t=0;t<crForest.size();++t)
				leaves[t] = crForest[t]->regressionIntegral( featureChans, maskIntegral, roi );

			if(!m_avg_votes){
				//go through the results
				for(unsigned int t=0;t<leaves.size();++t){

					if ( leaves[t]->trace > max_variance || leaves[t]->p < prob_th )
						continue;

					Vote v;

					//add the 3D location under the patch center to the vote for the head center
					v.vote[0] = leaves[t]->mean.at<float>(0) + rowX[roi.x + half_w];
					v.vote[1] = leaves[t]->mean.at<float>(1) + rowY[roi.x + half_w];
					v.vote[2] = leaves[t]->mean.at<float>(2) + rowZ[roi.x + half_w];

					//angles, leave as in the leaf
					v.vote[3] = leaves[t]->mean.at<float>(3);
					v.vote[4] = leaves[t]->mean.at<float>(4);
					v.vote[5] = leaves[t]->mean.at<float>(5);

					v.trace = &(leaves[t]->trace);
					v.conf = &(leaves[t]->p);

					votes.push_back(v);
				}
			}
			else{

				int v_count = 0;
				float pp = 0.f;
				float var = 0;

				vector< Vote > temp_votes;

				//go through the results
				for(unsigned int t=0;t<leaves.size();++t){

					//if ( leaves[t]->trace > max_variance || leaves[t]->p < prob_th )
					//	continue;

					pp+=leaves[t]->p;
					var+=leaves[t]->trace;

					v_count++;

					Vote v;

					//add the 3D location under the patch center to the vote for the head center
					v.vote[0] = leaves[t]->mean.at<float>(0) + rowX[roi.x + half_w];
					v.vote[1] = leaves[t]->mean.at<float>(1) + rowY[roi.x + half_w];
					v.vote[2] = leaves[t]->mean.at<float>(2) + rowZ[roi.x + half_w];

					//angles, leave as in the leaf
					v.vote[3] = leaves[t]->mean.at<float>(3);
					v.vote[4] = leaves[t]->mean.at<float>(4);
					v.vote[5] = leaves[t]->mean.at<float>(5);

					v.trace = &(leaves[t]->trace);
					v.conf = &(leaves[t]->p);

					temp_votes.push_back(v);
				}

				pp /= float(leaves.size());
				var /= float(leaves.size());

				if( pp >= .9f ){ //&& var < max_variance ){ //v_count >= leaves.size()/2 ){

					for(usint v=0;v<temp_votes.size();++v )
						if ( *(temp_votes[v].trace) < max_variance && *(temp_votes[v].conf) >= prob_th )
							votes.push_back( temp_votes[v] );

				}

			}
		} // end for x

	} // end for y

	delete [] channels;

}



void CRForestEstimator::do_clustering(const std::vector< Vote >& votes,
										float larger_radius_ratio,
										unsigned int max_clusters,
										Vec<float,POSE_SIZE>& temp_mean,
										std::vector< std::vector< const Vote* > >& temp_clusters,
										std::vector< Vec<float,POSE_SIZE> >& cluster_means,
										std::vector< std::vector< VoteIndex > >& cluster_votes_indeces ){


    //radius for clustering votes
    float large_radius = AVG_FACE_DIAMETER2/(larger_radius_ratio*larger_radius_ratio);

    //cluster using the head centers
    for(unsigned int l=0;l<votes.size();++l){

        bool found = false;
        float best_dist = FLT_MAX;
        unsigned int best_cluster = 0;

		//for each cluster
		for(unsigned int c=0; ( c<cluster_means.size() && found==false ); ++c){

			float norm = 0;
			for(int n=0;n<3;++n)
			    norm += (votes[l].vote[n]-cluster_means[c][n])*(votes[l].vote[n]-cluster_means[c][n]);

			//is the offset smaller than the radius?
			if( norm < large_radius ){

				best_cluster = c;
				found = true;

				 //add (pointer to) vote to the closest cluster (well, actually, the first cluster found within the distance)
				temp_clusters[best_cluster].push_back( &(votes[l]) );

				VoteIndex vi;
				vi.trace = votes[l].trace;
				vi.index = l;
				cluster_votes_indeces[best_cluster].push_back(vi);

			}

		}

		//create a new cluster
        if( !found && temp_clusters.size() < max_clusters ){

            vector< const Vote* > new_cluster;
            new_cluster.push_back( &(votes[l]) );
            temp_clusters.push_back( new_cluster );

            Vec<float,POSE_SIZE> vote( votes[l].vote );
            cluster_means.push_back( vote );

            std::vector< VoteIndex > votes_index;
			VoteIndex vi;
			vi.trace = votes[l].trace;
			vi.index = l;
			votes_index.push_back(vi);
			cluster_votes_indeces.push_back( votes_index );

        }


        //update the means every 10 votes only
		if ( l%10 == 0){

			for(unsigned int c=0; c<cluster_means.size(); ++c){

				if ( temp_clusters[c].size() > 0 ){

					 //update cluster's mean
					cluster_means[c] = 0;
					for(unsigned int i=0;i < temp_clusters[c].size(); ++i)
						((cluster_means[c])) = ((cluster_means[c])) + temp_clusters[c][i]->vote;

					float div = float(MAX(1,temp_clusters[c].size()));
					for(int n=0;n<POSE_SIZE;++n)
						cluster_means[c][n] /= div;

				}

			}

		}

    }

}


void CRForestEstimator::estimate( const Mat & im3D,
								   std::vector< cv::Vec<float,POSE_SIZE> >& means, //output
								   std::vector< std::vector< const Vote* > >& clusters,
								   std::vector< Vote >& votes, //all votes
								   int stride,
								   float max_variance,
								   float prob_th,
								   float larger_radius_ratio,
								   float smaller_radius_ratio,
								   bool verbose,
								   int threshold,
								   int max_no_faces
								   ){


    unsigned int max_clusters = 20;
    int max_ms_iterations = 10;

    //the actual regression
    do_regression(im3D, stride, max_variance, prob_th, votes );

    if(verbose)
        cout << endl << "votes : " << votes.size() << endl;

	vector< vector< const Vote* > > temp_clusters;
	vector< Vec<float,POSE_SIZE> > cluster_means;
	std::vector< std::vector< VoteIndex > > cluster_votes_indeces;

	Vec<float,POSE_SIZE> temp_mean;

	//get head clusters from the votes
	do_clustering( votes,
					larger_radius_ratio,
					max_clusters,
					temp_mean,
					temp_clusters,
					cluster_means,
					cluster_votes_indeces );

	if(verbose){

		cout << cluster_means.size() << " CLUSTERS ";
		for(unsigned int c = 0 ; c<cluster_means.size(); ++c)
			cout << temp_clusters[c].size() << " ";
		cout << endl;

	}

	usint max_iters = 10;
	usint max_sub_clusters = 8;

	//timeval c_time;
	//gettimeofday( &c_time, NULL );
	//int seed = c_time.tv_usec;
	//cout << "SEED " <<  seed << endl;
	cv::RNG cvRNG(100);

	std::vector< ClusterIndex > c_index;

	//std::vector< std::vector< const Vote* > > new_clusters; //after MS
	vector< Vec<float,POSE_SIZE> > new_means;
	//vector< ClusterIndex > c_index;

	int count = 0;
	float ms_radius = AVG_FACE_DIAMETER*AVG_FACE_DIAMETER/(smaller_radius_ratio*smaller_radius_ratio);

	//threshold defining if the cluster belongs to a head: it depends on the stride and on the number of trees
	int th = cvRound((double)threshold*crForest.size()/(double)(stride*stride));

	//for each cluster
	for(unsigned int c=0;c<temp_clusters.size();++c){

		//find sub-clusters
		cv::Mat_<int> cluster_inits;
		cluster_inits.create( max_sub_clusters , 1 );
		cvRNG.fill( cluster_inits, RNG::UNIFORM, Scalar(0), Scalar( temp_clusters[c].size() ) );

		vector< Vec<float,POSE_SIZE> > sub_means( max_sub_clusters );
		vector< vector< const Vote* > > sub_clusters( max_sub_clusters );
		std::vector< ClusterIndex > c_sub_index;

		//initialize clusters on random votes
		for(unsigned int sc=0;sc<max_sub_clusters;++sc){

			usint vote_idx = cluster_inits( sc );

			sub_clusters[sc].clear();
			if(sc==0)
				sub_means[sc] = cluster_means[c];
			else
				sub_means[sc] = temp_clusters[c][ vote_idx ]->vote;

			//now we have all the votes close to the seeding one inside the cluster, do MS
			int count = 0;

			//unsigned int num_sel_votes = joined_clusters[c].size();
			vector< const Vote* > new_sub_cluster;
			Vec<float,POSE_SIZE> temp_sub_mean;

			for(unsigned int it=0; it<max_iters; ++it){

				//count = 0;
				temp_sub_mean = 0;
				new_sub_cluster.clear();

				//for each vote in the cluster
				for(unsigned int idx=0; idx < temp_clusters[c].size(); ++idx){

					//compute distance
					float norm = 0;
					for(int n=0;n<3;++n)
						norm += ( temp_clusters[c][idx]->vote[n] - sub_means[sc][n] )*( temp_clusters[c][idx]->vote[n] - sub_means[sc][n] );
						//norm += ( joined_clusters[c][idx].vote[n]-sub_means[sc][n] )*( votes[idx].vote[n]-sub_means[sc][n] );

					//is it within the radius?
					if( norm < ms_radius ){

						//add the vote to the cluster
						temp_sub_mean = temp_sub_mean + temp_clusters[c][idx]->vote;
						//if(it == max_ms_iterations -1)
						//new_sub_cluster[ idx ] = 1;
						new_sub_cluster.push_back( temp_clusters[c][idx] );
						//count++;

					}

				}

				//update mean
				for(int n=0;n<POSE_SIZE;++n)
					temp_sub_mean[n] /= (float)MAX(1,new_sub_cluster.size());

				//how much did the mean move?
				float distance_to_previous_mean2 = 0;
				for(int n=0;n<3;++n)
					distance_to_previous_mean2 += (temp_sub_mean[n]-sub_means[sc][n])*(temp_sub_mean[n]-sub_means[sc][n]);

				sub_means[sc] = temp_sub_mean;

				//update the cluster
				sub_clusters[sc] = new_sub_cluster;

				//stop iterating if did not move much
				if( distance_to_previous_mean2 < 1  )
					break;

				count++;

			}

			ClusterIndex ci;
			ci.index = sc;
			ci.val = new_sub_cluster.size();
			c_sub_index.push_back(ci);

		}


		sort(c_sub_index.begin(), c_sub_index.end());
	
		int max_index = c_sub_index[c_sub_index.size()-1].index;

		temp_clusters[c] = sub_clusters[ max_index ];
		cluster_means[c] = sub_means[ max_index ];


		ClusterIndex ci;
		ci.index = c;
		ci.val = temp_clusters[c].size();
		c_index.push_back(ci);


	}


	//failed to find one face
	if( temp_clusters.size()==0 )
		return;

	sort(c_index.begin(), c_index.end());
	if(verbose)
		for(usint y=0;y<c_index.size();++y)
			cout << c_index[y].index << " " << c_index[y].val << endl;


	for( usint c = 0; c < MIN( temp_clusters.size() , max_no_faces ); ++c){

		int max_index = c_index[c_index.size()-1-c].index;

		if( temp_clusters[max_index].size() < th )
			break;

		vector< const Vote* > cluster;
		cluster_means[max_index] = 0;

		//for each vote in the cluster
		for(unsigned int k=0; k < temp_clusters[max_index].size(); k++ ){

			cluster_means[max_index] = cluster_means[max_index] + temp_clusters[max_index][k]->vote;
			cluster.push_back( temp_clusters[max_index][k] );

		}

		float div = (float)MAX(1,temp_clusters[max_index].size());
		for(int n=0;n<POSE_SIZE;++n)
			cluster_means[max_index][n] /= div;

		means.push_back( cluster_means[max_index] );
		clusters.push_back( cluster );

	}


}

