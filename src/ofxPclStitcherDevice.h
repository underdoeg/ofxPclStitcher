#ifndef OFXPCLSTITCHERDEVICE_H
#define OFXPCLSTITCHERDEVICE_H


#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "ofMain.h"
#include "ofxGui.h"

class ofxPclStitcher;

typedef pcl::PointXYZ ofxPclPoint;
typedef pcl::PointXYZRGBA ofxPclPointColor;

typedef pcl::PointCloud<ofxPclPoint> ofxPclCloud;
typedef pcl::PointCloud<ofxPclPointColor> ofxPclCloudColor;

typedef ofxPclCloud::Ptr ofxPclCloudPtr;
typedef ofxPclCloudColor::Ptr ofxPclCloudPtrColor;

typedef ofxPclCloud::ConstPtr ofxPclCloudConstPtr;
typedef ofxPclCloudColor::ConstPtr ofxPclCloudConstPtrColor;

class ofxPclStitcherDeviceNode: public ofNode
{
public:
	void customDraw() {
		ofNode::customDraw();
		float depth = 100;
		float spread = 50;
		ofLine(0, 0, 0, -spread, -spread, depth);
		ofLine(0, 0, 0, -spread, spread, depth);
		ofLine(0, 0, 0, spread, -spread, depth);
		ofLine(0, 0, 0, spread, spread, depth);
	}
};

class ofxPclStitcherDevice {
public:
	~ofxPclStitcherDevice();

	bool hasNewData();

	int id;

private:
	ofxPclStitcherDevice(string address, ofParameter<bool> doColors);
	void cloudCallbackColor(const ofxPclCloudConstPtrColor cloudIn);
	void cloudCallback(const ofxPclCloudConstPtr cloudIn);
	void copyCloudFromThread();
	void processCloud();
	void draw();
	void drawOverlay();

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

	//pcl filters
	pcl::PassThrough<ofxPclPoint> passThrough;
	pcl::PassThrough<ofxPclPointColor> passThroughColor;

	pcl::ApproximateVoxelGrid<ofxPclPoint> grid;
	pcl::ApproximateVoxelGrid<ofxPclPointColor> gridColor;

	ofxPclStitcherDeviceNode dNode;

	bool dataNew;

	//PARAMETERS
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

	ofParameter<bool> doDraw;
};

#endif // OFXPCLSTITCHERDEVICE_H
