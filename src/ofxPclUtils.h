#ifndef OFXPCLUTILS_H
#define OFXPCLUTILS_H
#include "ofxPclStitcherDevice.h";
#include <pcl/surface/gp3.h>


//OF UTILS
ofColor getColorForId(int id);

//EIGEN UTILS

//ofVec4f toOf(const Eigen::Vector4f& vec);

//PCL UTILS
void toOf(ofxPclCloudPtr & cloud, ofMesh & mesh, float xfactor, float yfactor, float zfactor, ofColor color=ofColor::white);
void toOf(ofxPclCloudPtrColor & cloud, ofMesh & mesh, float xfactor=1, float yfactor=1, float zfactor=-1, bool overrideColor = false, ofColor color=ofColor::white);
void addIndices(ofMesh & mesh, pcl::PolygonMesh& triangles);
#endif // OFXPCLUTILS_H
