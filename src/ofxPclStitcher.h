#ifndef OFXPCLSTITCHER_H
#define OFXPCLSTITCHER_H

#include <pcl/surface/concave_hull.h>
#include "ofxPclStitcherDevice.h"
#include "ofxGui.h"


class ofxPclStitcher
{
public:
	ofxPclStitcher();
	~ofxPclStitcher();

	void update();
	void draw();

	void setup(bool autoCreateDevices = true, bool doColors = false);
	ofxPclStitcherDevice* createDevice(string address);
	ofxPclStitcherDevice* createDevice(int number);
	ofxPclStitcherDevice* createDevice();

	void toggleDebug();

	ofParameter<bool> doDownsample;
	ofParameter<float> downsampleSize;
	ofParameter<bool> doCalibrate;
	ofParameter<float> doScale;
	ofParameter<float> concaveHullSize;

	ofMesh mesh;

private:
	typedef std::vector< ofPtr<ofxPclStitcherDevice> > DeviceList;
	DeviceList devices;
	unsigned int curDeviceNumber;
	ofParameterGroup parameters;
	ofParameter<bool> doColors;
	ofEasyCam cam;

	ofxPclCloudPtr cloud;
	ofxPclCloudPtrColor cloudColor;

	pcl::ApproximateVoxelGrid<ofxPclPoint> grid;
	pcl::ApproximateVoxelGrid<ofxPclPointColor> gridColor;

	pcl::ConcaveHull<ofxPclPoint> concaveHull;
	pcl::ConcaveHull<ofxPclPointColor> concaveHullColor;

	string settingsFilename;

	ofParameter<bool> doConcaveHull;

	ofxPanel gui;
};

#endif // OFXPCLSTITCHER_H
