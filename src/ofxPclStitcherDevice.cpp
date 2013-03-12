#include "ofxPclStitcherDevice.h"

ofxPclStitcherDevice::ofxPclStitcherDevice(string address, bool dc) {

	doColors = dc;

	try {
		/*
		interface = new pcl::OpenNIGrabber(kinectId);

		boost::function<void (const CloudConstPtr&)> f = boost::bind (&KinectWrapper::cloud_cb_, this, _1);

		interface->registerCallback(f);

		if(useCamera) {
			boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> fImg = boost::bind (&KinectWrapper::img_cb_, this, _1);
			interface->registerCallback(fImg);
		}

		interface->start();
		*/
	} catch(pcl::PCLException e) {
		cout << "SOMETHING WENT WRONG!! " << endl << e.detailedMessage() << endl;
	}

	//ofLog() << "created new kinect with id " << id;
}

ofxPclStitcherDevice::~ofxPclStitcherDevice() {
}
