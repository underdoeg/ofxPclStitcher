#ifndef OFXPCLUTILS_H
#define OFXPCLUTILS_H
#include "ofxPclStitcherDevice.h";

//OF UTILS
ofColor getColorForId(int id);

//EIGEN UTILS
Eigen::Matrix4f toEigen(ofMatrix4x4 m);
ofVec4f toOf(Eigen::Vector4f v);

//PCL UTILS
void toOf(ofxPclCloudPtr & cloud, ofMesh & mesh, float xfactor, float yfactor, float zfactor, ofColor color=ofColor::white);
void toOf(ofxPclCloudPtrColor & cloud, ofMesh & mesh, float xfactor=1, float yfactor=1, float zfactor=-1, bool overrideColor = false, ofColor color=ofColor::white);

#endif // OFXPCLUTILS_H
