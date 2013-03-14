#include "ofxPclUtils.h"

void toOf(ofxPclCloudPtr& cloud, ofMesh & mesh, float xfactor, float yfactor, float zfactor, ofColor color) {
	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_POINTS);
	mesh.getVertices().resize(cloud->points.size());
	mesh.getColors().resize(cloud->points.size());

	if(cloud->is_dense) {
		for(int i=0; i<cloud->points.size(); i++) {
			mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
		}
	} else {
		int i=0, n=0;
		for(int y=0; y<(int)cloud->height; y++) {
			for(int x=0; x<(int)cloud->width; x++) {
				if(isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z)) {
					mesh.getVertices()[i] = ofVec3f(float(x)/float(cloud->width)*xfactor,float(y)/float(cloud->height)*yfactor,0);
					mesh.getColors()[i] = ofColor(0,0,0,0);
					n++;
				} else {
					mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
					mesh.getColors()[i] = color;
				}
				i++;
			}
		}
	}
}

void toOf(ofxPclCloudPtrColor& cloud, ofMesh & mesh, float xfactor, float yfactor, float zfactor, bool overrideColor, ofColor color) {
	mesh.setMode(OF_PRIMITIVE_POINTS);
	mesh.getVertices().resize(cloud->points.size());
	mesh.getColors().resize(cloud->points.size());
	int i=0, n=0;
	if(cloud->is_dense) {
		for(int i=0; i<cloud->points.size(); i++) {
			mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
			if(overrideColor)
				mesh.getColors()[i] = color;
			else
				mesh.getColors()[i] = ofColor(cloud->points[i].r,cloud->points[i].g,cloud->points[i].b);
		}
	} else {
		for(int y=0; y<(int)cloud->height; y++) {
			for(int x=0; x<(int)cloud->width; x++) {
				if(isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z)) {
					mesh.getVertices()[i] = ofVec3f(float(x)/float(cloud->width)*xfactor,float(y)/float(cloud->height)*yfactor,0);
					mesh.getColors()[i] = ofColor(0,0,0,0);
					n++;
				} else {
					mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
					if(overrideColor)
						mesh.getColors()[i] = color;
					else
						mesh.getColors()[i] = ofColor(cloud->points[i].r,cloud->points[i].g,cloud->points[i].b);
				}
				i++;
				//cout << ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor) << endl;
			}
		}
	}
}
