#include "ofxPclStitcherDevice.h"
#include <pcl/common/transforms.h>

#include "ofxPclUtils.h"

int ofxPclStitcherDevice::curId = 0;

ofxPclStitcherDevice::ofxPclStitcherDevice(string address, ofParameter<bool> dc) {

	id = curId;
	curId++;

	color = getColorForId(id);

	cropZ.set("CROP Z", 100, 0, 800);

	float minTrans = -50;
	float maxTrans = minTrans*-1;;
	translateX.set("TRANSLATE X", 0, minTrans, maxTrans);
	translateY.set("TRANSLATE Y", 0, minTrans, maxTrans);
	translateZ.set("TRANSLATE Z", 0, minTrans, maxTrans);

	float minRot = -180;
	float maxRot = 180;
	rotationX.set("ROTATION X", 0, minRot, maxRot);
	rotationY.set("ROTATION Y", 0, minRot, maxRot);
	rotationZ.set("ROTATION Z", 0, minRot, maxRot);

	parameters.setName("PCL DEVICE "+ofToString(id));
	parameters.add(cropZ);
	parameters.add(translateX);
	parameters.add(translateY);
	parameters.add(translateZ);
	parameters.add(rotationX);
	parameters.add(rotationY);
	parameters.add(rotationZ);

	doColors = dc;

	//TODO: this could be optimized, since we don't need color and non color instantiated at the same time
	cloud = ofxPclCloudPtr(new ofxPclCloud());
	cloudThread = ofxPclCloudPtr(new ofxPclCloud());
	cloudColor = ofxPclCloudPtrColor(new ofxPclCloudColor());
	cloudThreadColor = ofxPclCloudPtrColor(new ofxPclCloudColor());

	try {
		interface = new pcl::OpenNIGrabber(address);

		if(doColors) {
			boost::function<void (const ofxPclCloudConstPtrColor&)> f = boost::bind (&ofxPclStitcherDevice::cloudCallbackColor, this, _1);
			interface->registerCallback(f);
		} else {
			boost::function<void (const ofxPclCloudConstPtr&)> f = boost::bind (&ofxPclStitcherDevice::cloudCallback, this, _1);
			interface->registerCallback(f);
		}

		interface->start();

	} catch(pcl::PCLException e) {
		ofLogError() << e.detailedMessage() << endl;
	}

	ofLog() << "CREATED NEW PCL DEVICE: " << address;
}

ofxPclStitcherDevice::~ofxPclStitcherDevice() {
	interface->stop();
}

void ofxPclStitcherDevice::cloudCallback(const ofxPclCloudConstPtr cloudIn) {
	mutex.lock();
	pcl::copyPointCloud(*cloudIn, *cloudThread);
	mutex.unlock();
}

void ofxPclStitcherDevice::cloudCallbackColor(const ofxPclCloudConstPtrColor cloudIn) {
	mutex.lock();
	pcl::copyPointCloud(*cloudIn, *cloudThreadColor);
	mutex.unlock();
}

void ofxPclStitcherDevice::copyCloudFromThread() {
	if(doColors) {
		mutex.lock();
		pcl::copyPointCloud(*cloudThreadColor, *cloudColor);
		mutex.unlock();
	} else {
		mutex.lock();
		pcl::copyPointCloud(*cloudThread, *cloud);
		mutex.unlock();
	}
}

void ofxPclStitcherDevice::processCloud() {

	//Z filtering
	if(doColors) {
		passThroughColor.setFilterFieldName ("z");
		passThroughColor.filter (*cloudColor);
		passThroughColor.setFilterLimits (0.0, cropZ/scale);
		passThroughColor.setInputCloud (cloudColor);
	} else {
		passThrough.setFilterFieldName ("z");
		passThrough.filter (*cloud);
		passThrough.setFilterLimits (0.0, cropZ/scale);
		passThrough.setInputCloud (cloud);
	}



	//do downsampling
	if(downsample) {
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

	//apply matrix transforms
	ofMatrix4x4 matrix;
	matrix.translate(translateX/scale, translateY/scale, translateZ/scale);
	matrix.rotate(rotationX, 1, 0, 0);
	matrix.rotate(rotationY, 0, 1, 0);
	matrix.rotate(rotationZ, 0, 0, 1);
	matrix.scale(-1, -1, 1);

	if(debug) {
		ofMatrix4x4 matrixDebug;
		matrixDebug.translate(translateX, translateY, translateZ);
		matrixDebug.rotate(rotationX, 1, 0, 0);
		matrixDebug.rotate(rotationY, 0, 1, 0);
		matrixDebug.rotate(rotationZ, 0, 0, 1);
		matrixDebug.scale(-1, -1, 1);
		dNode.setTransformMatrix(matrixDebug);
	}

	//TODO: rotation
	//matrix.scale(1, -1, -1);

	if(doColors)
		pcl::transformPointCloud(*cloudColor, *cloudColor, toEigen(matrix));
	else
		pcl::transformPointCloud(*cloud, *cloud, toEigen(matrix));

	if(debug) {
		if(doColors) {
			toOf(cloudColor, mesh, scale, scale, scale, true, color);
		} else {
			toOf(cloud, mesh, scale, scale, scale, color);
		}
	}
}

void ofxPclStitcherDevice::draw() {
	if(debug) {
		ofSetColor(color);
		mesh.draw();
		dNode.draw();
	}
}

void ofxPclStitcherDevice::drawOverlay() {
	if(debug) {
	}
}
