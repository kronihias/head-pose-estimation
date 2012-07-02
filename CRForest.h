#pragma once

#ifndef CRForest_H
#define CRForest_H

#include "CRTree.h"
#include <stdio.h>
#include <vector>

class CRForest {
  public:
    // Constructor
    CRForest(int trees = 0) {
      vTrees.resize(trees);
    }

    // Destructor
    ~CRForest() {
      for(std::vector<CRTree*>::iterator it = vTrees.begin(); it != vTrees.end(); ++it)
    	  delete *it; // delete pointers
      vTrees.clear(); // specialized routine for clearing trees
    }

    // Set/Get functions
    int getSize() const {return vTrees.size();} // get size of trees
    int getDepth() const {return vTrees[0]->getDepth();}
    int getPatchWidth(){ return vTrees[0]->getPatchWidth(); }
    int getPatchHeight(){ return vTrees[0]->getPatchHeight(); }
    int getNoChans(){ return vTrees[0]->getNoChannels(); }

    std::vector< const LeafNode* > regressionIntegral( const std::vector< cv::Mat >& patch, const cv::Mat& nonZeros, const cv::Rect& roi ) const;

	bool loadForest(const char* filename);

    // Trees
    std::vector<CRTree*> vTrees;


};

// Regression
inline std::vector<const LeafNode*> CRForest::regressionIntegral( const std::vector< cv::Mat >& patch, const cv::Mat& nonZeros, const cv::Rect& roi ) const {

	std::vector<const LeafNode*> res;
	for(int i=0; i<(int)vTrees.size(); ++i)
		res.push_back( vTrees[i]->regressionIntegral(patch,nonZeros,roi) );
	return res;
}


inline bool CRForest::loadForest(const char* filename) {

	char buffer[200];
	bool success = true;
	for(unsigned int i=0; i<vTrees.size(); ++i) {
		sprintf(buffer,"%s%03d.bin",filename,i);
		vTrees[i] = new CRTree();
		success &= vTrees[i]->loadTree(buffer);
	}
	return success;
}


#endif
