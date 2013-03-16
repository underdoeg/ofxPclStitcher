#include "ofxPclUtils.h"

ofColor getColorForId(int id) {
	switch(id) {
	case 0:
		return ofColor(180, 0, 0);
	case 1:
		return ofColor(0, 180, 0);
	case 2:
		return ofColor(180, 180, 0);
	case 3:
		return ofColor(0, 0, 180);
	case 4:
		return ofColor(0, 180, 180);
	case 5:
		return ofColor(180, 0, 180);
	}
	return ofColor(255, 255, 255);
}

Eigen::Matrix4f toEigen(ofMatrix4x4 m) {
	Eigen::Matrix4f ret;
	ret << m(0,0), m(1,0), m(2,0), m(3,0),
	    m(0,1), m(1,1), m(2,1), m(3,1),
	    m(0,2), m(1,2), m(2,2), m(3,2),
	    m(0,3), m(1,3), m(2,3), m(3,3);
	return ret;
};

ofVec4f toOf(Eigen::Vector4f v) {
	return ofVec4f(v[0], v[1], v[2], v[3]);
}

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
	mesh.clear();
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
