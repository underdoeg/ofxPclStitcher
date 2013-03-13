#include "ofxPclStitcherDevice.h"

ofxPclStitcherDevice::ofxPclStitcherDevice(string address, bool dc) {

	doColors = dc;

	try {

		interface = new pcl::OpenNIGrabber(address);


		if(doColors){
			boost::function<void (const ofxPclCloudConstPtrColor&)> f = boost::bind (&ofxPclStitcherDevice::cloudCallbackColor, this, _1);
			interface->registerCallback(f);
		}else{
			boost::function<void (const ofxPclCloudConstPtr&)> f = boost::bind (&ofxPclStitcherDevice::cloudCallback, this, _1);
			interface->registerCallback(f);
		}



		/*
		if(useCamera) {
			boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> fImg = boost::bind (&KinectWrapper::img_cb_, this, _1);
			interface->registerCallback(fImg);
		}
		*/

		interface->start();

	} catch(pcl::PCLException e) {
		ofLogError() << e.detailedMessage() << endl;
	}

	//ofLog() << "created new kinect with id " << id;
}

ofxPclStitcherDevice::~ofxPclStitcherDevice() {
}

void ofxPclStitcherDevice::cloudCallback(const ofxPclCloudConstPtr cloudIn)
{
	ofLog() << "CLOUD IN";
}

void ofxPclStitcherDevice::cloudCallbackColor(const ofxPclCloudConstPtrColor cloudIn)
{
	ofLog() << "CLOUD IN";
}
