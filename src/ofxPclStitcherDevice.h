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

private:
	ofxPclStitcherDevice(string address, ofParameter<bool> doColors);
	void cloudCallbackColor(const ofxPclCloudConstPtrColor cloudIn);
	void cloudCallback(const ofxPclCloudConstPtr cloudIn);
	void copyCloudFromThread();
	void processCloud();

	pcl::OpenNIGrabber* interface;

	friend class ofxPclStitcher;

	ofMutex mutex;

	ofxPclCloudPtr cloudThread;
	ofxPclCloudPtrColor cloudThreadColor;

	ofxPclCloudPtr cloud;
	ofxPclCloudPtrColor cloudColor;

	ofMesh mesh;

	ofParameterGroup parameters;
	ofParameter<bool> doColors;
	ofParameter<float> cropZ;

	ofParameter<bool> downsample;
	ofParameter<float> downsampleSize;
	ofParameter<float> scale;
	ofParameter<bool> debug;
};

#endif // OFXPCLSTITCHERDEVICE_H
