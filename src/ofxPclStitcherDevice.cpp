#include "ofxPclStitcherDevice.h"
#include <pcl/common/transforms.h>

#include "ofxPclUtils.h"

int ofxPclStitcherDevice::curId = 0;

ofxPclStitcherDevice::ofxPclStitcherDevice(string address, ofParameter<bool> dc) {

	dataNew = false;

	doDraw.set("DRAW", true);

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
	parameters.add(doDraw);
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

	ofLog() << "CREATING DEVICE WITH ADDRESS " << address;

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
	dataNew = true;
	mutex.unlock();
}

void ofxPclStitcherDevice::cloudCallbackColor(const ofxPclCloudConstPtrColor cloudIn) {
	mutex.lock();
	dataNew = true;
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
	mutex.lock();
	dataNew = false;
	mutex.unlock();
}

void ofxPclStitcherDevice::processCloud() {
	if(debug && !doDraw)
		return;

	//Z filtering
	if(doColors) {
		passThroughColor.setFilterFieldName ("z");
		passThroughColor.filter (*cloudColor);
		passThroughColor.setFilterLimits (0.0, cropZ/scale);
		passThroughColor.setInputCloud (cloudColor);
	} else {
		ofxPclCloud cloud_temp;
		passThrough.setInputCloud (cloud);
		passThrough.setFilterFieldName ("z");
		passThrough.setFilterLimits (0.0, cropZ/scale);
		passThrough.filter (cloud_temp);
		pcl::copyPointCloud(cloud_temp, *cloud);
		//
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
	/*
	ofQuaternion quat;
	ofVec3f Znormal(0, 0, 1);
	ofVec3f Xnormal(1, 0, 0);
	ofVec3f Ynormal(1, 0, 1);
	ofQuaternion qr (rotationZ, Znormal); // quat roll.
	ofQuaternion qp (rotationX, Xnormal); // quat pitch.
	ofQuaternion qh (rotationY, Ynormal); // quat heading or yaw.
	quat = qr * qp * qh;
	*/
	dNode.resetTransform();
	dNode.tilt(rotationX);
	dNode.pan(rotationY);
	dNode.roll(rotationZ);
	dNode.setPosition(translateX, translateY, translateZ);

	ofMatrix4x4 matrix = dNode.getGlobalTransformMatrix().getInverse();
	matrix.setTranslation(translateX/scale, translateY/scale, translateZ/scale);
	//matrix.scale(-1, -1, 1);

	/*
	if(debug) {
		ofMatrix4x4 matrixNode;
		matrixNode.setRotate(quat);
		matrixNode.translate(translateX, translateY, translateZ);
		matrixNode.scale(-1, -1, 1);
		dNode.setTransformMatrix(matrixNode);
	}
	*/

	//TODO: rotation
	//matrix.scale(1, -1, -1);

	Eigen::Matrix4f eigenMat;
	eigenMat << matrix(0,0), matrix(1,0), matrix(2,0), matrix(3,0),
	         matrix(0,1), matrix(1,1), matrix(2,1), matrix(3,1),
	         matrix(0,2), matrix(1,2), matrix(2,2), matrix(3,2),
	         matrix(0,3), matrix(1,3), matrix(2,3), matrix(3,3);

	if(doColors)
		pcl::transformPointCloud(*cloudColor, *cloudColor, eigenMat);
	else
		pcl::transformPointCloud(*cloud, *cloud, eigenMat);

	if(debug) {
		if(doColors) {
			toOf(cloudColor, mesh, scale, scale, scale, true, color);
		} else {
			toOf(cloud, mesh, scale, scale, scale, color);
		}
	}
}

void ofxPclStitcherDevice::draw() {
	if(!doDraw)
		return;
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

bool ofxPclStitcherDevice::hasNewData() {
	bool ret;
	mutex.lock();
	ret = dataNew;
	mutex.unlock();
	return ret;
}
