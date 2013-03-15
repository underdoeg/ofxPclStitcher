#ifndef OFXPCLSTITCHERDEVICE_H
#define OFXPCLSTITCHERDEVICE_H

#include <XnCppWrapper.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>

#include "ofMain.h"

class ofxPclStitcher;

typedef pcl::PointXYZ ofxPclPoint;
typedef pcl::PointXYZRGBA ofxPclPointColor;

typedef pcl::PointCloud<ofxPclPoint> ofxPclCloud;
typedef pcl::PointCloud<ofxPclPointColor> ofxPclCloudColor;

typedef ofxPclCloud::Ptr ofxPclCloudPtr;
typedef ofxPclCloudColor::Ptr ofxPclCloudPtrColor;

typedef ofxPclCloud::ConstPtr ofxPclCloudConstPtr;
typedef ofxPclCloudColor::ConstPtr ofxPclCloudConstPtrColor;


class ofxPclStitcherDevice {
public:
	~ofxPclStitcherDevice();

	int id;

private:
	ofxPclStitcherDevice(string address, ofParameter<bool> doColors);
	void cloudCallbackColor(const ofxPclCloudConstPtrColor cloudIn);
	void cloudCallback(const ofxPclCloudConstPtr cloudIn);
	void copyCloudFromThread();
	void processCloud();
	void draw();

	pcl::OpenNIGrabber* interface;

	friend class ofxPclStitcher;

	ofMutex mutex;

	ofxPclCloudPtr cloudThread;
	ofxPclCloudPtrColor cloudThreadColor;

	ofxPclCloudPtr cloud;
	ofxPclCloudPtrColor cloudColor;

	static int curId;

	ofColor color;
	ofMesh mesh;

	ofParameterGroup parameters;
	ofParameter<bool> doColors;
	ofParameter<float> cropZ;

	ofParameter<float> translateX;
	ofParameter<float> translateY;
	ofParameter<float> translateZ;

	ofParameter<float> rotationX;
	ofParameter<float> rotationY;
	ofParameter<float> rotationZ;

	ofParameter<bool> downsample;
	ofParameter<float> downsampleSize;
	ofParameter<float> scale;
	ofParameter<bool> debug;
};

#endif // OFXPCLSTITCHERDEVICE_H
