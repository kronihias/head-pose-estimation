/*
	// Authors: G. Fanelli, J. Gall, BIWI, ETH Zurich
// Email: fanelli@vision.ee.ethz.ch, gall@vision.ee.ethz.ch
*/

#include "CRTree.h"
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>


using namespace std;
using namespace cv;


void node::write( std::ofstream& os ){

	os << depth << " " << test_set << " " << leaf_set << " ";

	if(test_set){

		os << test.th << " " << test.ch << " ";
		os << test.f1.x << " "<< test.f1.y << " " << test.f1.width << " " << test.f1.height << " ";
		os << test.f2.x << " " << test.f2.y << " " << test.f2.width << " " << test.f2.height << endl;

		left_child->write( os );
		right_child->write( os );
	}
	if(leaf_set){

		os << leaf.p << " " << leaf.trace << " ";
		for (usint i=0;i<POSE_SIZE;++i)
			os << leaf.mean(i) << " ";
		os << endl;

	}

}


bool node::read( std::ifstream& is ){

	int aux = 0;
	is >> depth >> test_set >> leaf_set;

	if(test_set){

		is >> test.th >> test.ch;
		is >> test.f1.x >> test.f1.y >> test.f1.width >> test.f1.height;
		is >> test.f2.x >> test.f2.y >> test.f2.width >> test.f2.height;

		left_child = new node();
		left_child->read( is );

		right_child = new node();
		right_child->read( is );

	}
	//cout << depth << " " << test_set << " " << leaf_set << endl;
	if(leaf_set){

		is >> leaf.p >> leaf.trace;
		leaf.mean.create(POSE_SIZE,1);
		for (usint i=0;i<POSE_SIZE;++i)
			is >> leaf.mean(i);

	}

	if (is.good())
		return true;
	else
		return false;

}



bool CRTree::read_tree(const std::string& fname ){

	bool success = true;
	std::ifstream ifs(fname.c_str());

	if (!ifs.good())
		success = false;

	m_root = new node;

	ifs >> m_p_w >> m_p_h;

	//cout << m_p_w << " " << m_p_h << endl;

	success = m_root->read( ifs );

	ifs.close();
	return success;

}


