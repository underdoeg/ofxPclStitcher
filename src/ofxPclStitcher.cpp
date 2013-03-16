#include "ofxPclStitcher.h"

#include "ofxPclUtils.h"

ofxPclStitcher::ofxPclStitcher():curDeviceNumber(1) {
}

ofxPclStitcher::~ofxPclStitcher() {
}

void ofxPclStitcher::setup(bool autoCreateDevices, bool dc) {

	doCalibrate.set("CALIBRATE", false);

	doColors = dc;
	doDownsample.set("DOWNSAMPLE", true);
	downsampleSize.set("DOWNSAMPLE SIZE", .3, 0.001, .1);
	doScale.set("SCALE", 100, 0.1, 1000);

	doConcaveHull.set("CONCAVE HULL", false);
	concaveHullSize.set("CONCAVE HULL SIZE", .1, .0001, .5);

	doTriangulation.set("TRIANGULATION", false);
	triangulationRadius.set("TRIANGULATION RADIUS", .03, .0001, .5);

	settingsFilename = "pclStitcherSettings.xml";

	gui.setup("PCL STITCHER SETTINGS", settingsFilename);
	gui.add(doDownsample);
	gui.add(downsampleSize);
	gui.add(doTriangulation);
	gui.add(triangulationRadius);
	//gui.add(doConcaveHull);
	gui.loadFromFile(settingsFilename);
	gui.setPosition(10, 40);

	cloud = ofxPclCloudPtr(new ofxPclCloud());
	cloudColor = ofxPclCloudPtrColor(new ofxPclCloudColor());
	normals =  pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

//	searchTree = pcl::search::KdTree<ofxPclPoint>::Ptr(new pcl::search::kdTree<ofxPclPoint>);

	if(autoCreateDevices) {
		unsigned int numDevices = 0;

		//enumerate devices done with openni because AFAIK PCL does not offer device enumeration?
		xn::Context context;
		context.Init();
		xn::NodeInfoList deviceNodes;

		XnStatus status = context.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, deviceNodes);
		if(status != XN_STATUS_OK) {
			ofLogError() << xnGetStatusString(status) << endl;
		}

		for (xn::NodeInfoList::Iterator nodeIt = deviceNodes.Begin(); nodeIt != deviceNodes.End (); ++nodeIt) {
			numDevices++;
		}

		context.Release();

		ofLogNotice() << "FOUND " << numDevices << " DEVICE(S)";

		for(unsigned int i=0; i<numDevices; i++) {
			createDevice();
		}
	}
}

ofxPclStitcherDevice* ofxPclStitcher::createDevice() {
	curDeviceNumber++;
	return createDevice(curDeviceNumber-1);
}

ofxPclStitcherDevice* ofxPclStitcher::createDevice(int number) {
	return createDevice("#"+ofToString(number));
}

ofxPclStitcherDevice* ofxPclStitcher::createDevice(string address) {
	ofxPclStitcherDevice* device = new ofxPclStitcherDevice(address, doColors);
	device->downsample.makeReferenceTo(doDownsample);
	device->downsampleSize.makeReferenceTo(downsampleSize);
	device->scale.makeReferenceTo(doScale);
	device->debug.makeReferenceTo(doCalibrate);
	gui.add(device->parameters);
	gui.loadFromFile(settingsFilename);
	devices.push_back(ofPtr<ofxPclStitcherDevice>(device));
	return device;
}

void ofxPclStitcher::update() {
	//copy clouds over from thread
	for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
		(*it)->copyCloudFromThread();
	}

	for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
		(*it)->processCloud();
	}

	if(!doCalibrate) { //don't create the final mesh in debug mode to save performance

		cloud->clear();
		cloudColor->clear();

		for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
			if(doColors)
				*cloudColor += *(*it)->cloudColor;
			else
				*cloud += *(*it)->cloud;
		}

		//downsample if needed
		if(doDownsample) {
			if(doColors) {
				gridColor.setLeafSize(downsampleSize, downsampleSize, downsampleSize);
				gridColor.setInputCloud(cloudColor);
				gridColor.filter(*cloudColor);
			} else {
				grid.setLeafSize(downsampleSize, downsampleSize, downsampleSize);
				grid.setInputCloud(cloud);
				grid.filter(*cloud);
			}
		}

		//construct the concave hull
		if(doConcaveHull){
			if(doColors) {
				concaveHullColor.setInputCloud(cloudColor);
				concaveHullColor.setAlpha(concaveHullSize);
				concaveHullColor.reconstruct(*cloudColor);
			} else {
				concaveHull.setInputCloud(cloud);
				concaveHull.setAlpha(concaveHullSize);
				concaveHull.reconstruct(*cloud);
			}
		}


		if(doColors) {
			toOf(cloudColor, mesh, doScale, doScale, doScale);
		} else {
			toOf(cloud, mesh, doScale, doScale, doScale);
		}

	}

}

void ofxPclStitcher::draw() {
	ofPushStyle();
	cam.begin();
	glEnable(GL_DEPTH_TEST);
	ofEnableAlphaBlending();
	for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
		(*it)->draw();
	}

	if(!doCalibrate)
		mesh.draw();

	cam.end();
	glDisable(GL_DEPTH_TEST);

	if(doCalibrate)
		gui.draw();

	for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
		(*it)->drawOverlay();
	}

	ofPopStyle();

}

void ofxPclStitcher::toggleDebug() {
	doCalibrate = !doCalibrate;
}
