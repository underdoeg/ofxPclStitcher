#ifndef OFXPCLUTILS_H
#define OFXPCLUTILS_H
#include "ofxPclStitcherDevice.h";

void toOf(ofxPclCloudPtr & cloud, ofMesh & mesh, float xfactor, float yfactor, float zfactor, ofColor color=ofColor::white);
void toOf(ofxPclCloudPtrColor & cloud, ofMesh & mesh, float xfactor=1, float yfactor=1, float zfactor=-1, bool overrideColor = false, ofColor color=ofColor::white);

#endif // OFXPCLUTILS_H
