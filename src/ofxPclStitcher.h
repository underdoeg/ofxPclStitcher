#ifndef OFXPCLSTITCHER_H
#define OFXPCLSTITCHER_H

#include <pcl/surface/concave_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>


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
	ofParameter<float> scale;
	ofParameter<float> concaveHullSize;
	ofParameter<bool> doTriangulation;
	ofParameter<float> triangulationRadius;
	ofParameter<bool> doNoiseReduction;

	ofEvent<ofEventArgs> onPreprocess;

	ofEvent<ofEventArgs> onCalibrateDraw;

	ofMesh mesh;

	ofxPclCloudPtr cloud;
	ofxPclCloudPtrColor cloudColor;
	ofxPanel gui;

private:
	void draw3d();
	void keyReleased(ofKeyEventArgs& e);
	void keyPressed(ofKeyEventArgs& e);
	typedef std::vector< ofPtr<ofxPclStitcherDevice> > DeviceList;
	DeviceList devices;
	unsigned int curDeviceNumber;
	ofParameterGroup parameters;
	ofParameter<bool> doColors;
	ofEasyCam easyCam;
	ofCamera camTop;
	ofCamera camLeft;
	ofCamera camFront;
	ofCamera* cam;

	pcl::ApproximateVoxelGrid<ofxPclPoint> grid;
	pcl::ApproximateVoxelGrid<ofxPclPointColor> gridColor;

	pcl::ConcaveHull<ofxPclPoint> concaveHull;
	pcl::ConcaveHull<ofxPclPointColor> concaveHullColor;

	pcl::NormalEstimation<ofxPclPoint, pcl::Normal> normalEstimation;
	pcl::NormalEstimation<ofxPclPointColor, pcl::Normal> normalEstimationColor;

	pcl::search::KdTree<ofxPclPoint>::Ptr searchTree;
	pcl::search::KdTree<ofxPclPointColor>::Ptr searchTreeColor;



	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh polygonMesh;

	pcl::PointCloud<pcl::Normal>::Ptr normals;

	string settingsFilename;

	ofParameter<bool> doConcaveHull;

	ofParameter<bool> drawGrid;

	int guiWidth;

	int firstDraw; //hack for ofEasyCam
};

#endif // OFXPCLSTITCHER_H
