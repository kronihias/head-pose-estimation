/*
// Authors: G. Fanelli, J. Gall, BIWI, ETH Zurich
// Email: fanelli@vision.ee.ethz.ch, gall@vision.ee.ethz.ch
*/

#ifndef CRTree_H
#define CRTree_H

#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define POSE_SIZE 6
#define SCALE 1000.f
#define AVG_FACE_DIAMETER 236.4f
#define AVG_FACE_DIAMETER2 55884.96f
#define YAW_RANGE 190.f
#define usint unsigned int

// Structure for the leafs
class leaf_data {

public:

	leaf_data() { }
	~leaf_data(){ }
	std::vector< cv::Mat_<float> > offsets;

	cv::Mat_<float> mean, cov;
	float trace, p;

	bool operator<(const leaf_data& a) const { return trace < a.trace; }
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & offsets;
		ar & mean;
		ar & cov;
		ar & trace;
		ar & p;
	}

};

class binary_test {

public:

	binary_test() { }
	binary_test( const binary_test& test ) {

		th = test.th;
		ch = test.ch;

		f1 = test.f1;
		f2 = test.f2;
	}

	binary_test& operator=(const binary_test& test) {

	  if(this == &test)
		return *this;

	  this->th = test.th;
	  this->ch = test.ch;

	  this->f1 = test.f1;
	  this->f2 = test.f2;

	  return *this;
	}


	~binary_test(){ }

	unsigned int ch;
	int th;
	cv::Rect_<unsigned int> f1, f2;

	friend std::ostream &operator<<( std::ostream &stream, const binary_test& o);

};

class node {

public:

	node(){
		test_set = false;
		leaf_set = false;
		left_child = 0;
		right_child = 0;
		depth = 0;
	}

	~node(){
		if(left_child)
			delete left_child;
		if(right_child)
			delete right_child;
		}

	bool test_set, leaf_set;
	binary_test test;
	leaf_data leaf;

	int depth;
	node* left_child, *right_child;

	bool read( std::ifstream& is );
	void write( std::ofstream& os );

};

/*
struct train_parameters{

	unsigned int p_w; //size of the patch
	unsigned int p_h;
	unsigned int max_inner_p;

	unsigned int min_no_patches;
	unsigned int max_depth;
	unsigned int no_tests;
	unsigned int no_thresholds;

	int no_channels_code;
	float lambda;

	unsigned int no_threads;

	int max_patches_per_node;

	double m_min_det;

	void print( std::ostream& stream ) const {

		stream << "patch size " << p_w << " " << p_h  << "\n";
		stream << "min patches " << min_no_patches << "\n";

		stream << "max depth " << max_depth << "\n";
		stream << "no tests " << no_tests << "\n";
		stream << "no thresholds " << no_thresholds << "\n";

		stream << "chans (code) " << no_channels_code << "\n";
		stream << "LAMBDA " << lambda << "\n";
		stream << "no threads " << no_threads << "\n";
		stream << "min_det " << m_min_det << "\n";

	}

};*/

class CRTree {

public:

	CRTree(	){ m_root = 0; };

	~CRTree() { delete m_root; };

	const leaf_data* regressionIntegral(const std::vector< cv::Mat >& patch,
														const cv::Mat& nonZeros,
														const cv::Rect& roi );

    node* m_root;

	bool read_tree( const std::string& fname );

	unsigned int m_p_w, m_p_h;

};

inline const leaf_data* CRTree::regressionIntegral(const std::vector< cv::Mat >& patch,
													const cv::Mat& nonZeros,
													const cv::Rect& roi ) {
	// pointer to current node
	const node* curr_node = m_root;
	bool test_result = false;

	while( curr_node->leaf_set != true ) {

		//std::cout << curr_node->leaf_set << " " << curr_node->test_set << " " << curr_node->right_child << " " << curr_node->left_child << std::endl;
		const binary_test* test = &(curr_node->test);

		const cv::Mat ptC = patch[ test->ch ];

		int xa1 = roi.x + test->f1.x;	int xa2 = xa1 + test->f1.width;
		int ya1 = roi.y + test->f1.y;	int ya2 = ya1 + test->f1.height;

		int xb1 = roi.x + test->f2.x;	int xb2 = xb1 + test->f2.width;
		int yb1 = roi.y + test->f2.y;	int yb2 = yb1 + test->f2.height;

		double mz1 = ( ptC.at<double>(ya1,xa1) +
					   ptC.at<double>(ya2,xa2) -
					   ptC.at<double>(ya2,xa1) -
					   ptC.at<double>(ya1,xa2) ) /
					   (double)MAX(1, nonZeros.at<double>(ya1,xa1) +
									   nonZeros.at<double>(ya2,xa2) -
									   nonZeros.at<double>(ya2,xa1) -
									   nonZeros.at<double>(ya1,xa2));


		double mz2 = ( ptC.at<double>(yb1,xb1) +
					   ptC.at<double>(yb2,xb2) -
					   ptC.at<double>(yb2,xb1) -
					   ptC.at<double>(yb1,xb2) ) /
					   (double)MAX(1, nonZeros.at<double>(yb1,xb1) +
									   nonZeros.at<double>(yb2,xb2) -
									   nonZeros.at<double>(yb2,xb1) -
									   nonZeros.at<double>(yb1,xb2));

		test_result = ( mz1 - mz2 ) > double( test->th );

		if (test_result)
			curr_node = curr_node->right_child;
		else
			curr_node = curr_node->left_child;

	}

	return & curr_node->leaf;


}


#endif

