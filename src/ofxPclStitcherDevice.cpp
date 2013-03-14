#include "ofxPclStitcherDevice.h"

ofxPclStitcherDevice::ofxPclStitcherDevice(string address, ofParameter<bool> dc) {

	cropZ.set("CROP Z", 100, 0, 800);

	parameters.add(cropZ);

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
	if(doColors){
		mutex.lock();
		pcl::copyPointCloud(*cloudThreadColor, *cloudColor);
		mutex.unlock();
	}else{
		mutex.lock();
		pcl::copyPointCloud(*cloudThread, *cloud);
		mutex.unlock();
	}
}

void ofxPclStitcherDevice::processCloud()
{

}
