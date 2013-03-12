#ifndef OFXPCLSTITCHERDEVICE_H
#define OFXPCLSTITCHERDEVICE_H

#include <XnCppWrapper.h>
#include <pcl/io/openni_grabber.h>

#include "ofMain.h"

class ofxPclStitcher;

class ofxPclStitcherDevice {
public:
	~ofxPclStitcherDevice();

private:
	ofxPclStitcherDevice(string address, bool doColors);

	pcl::OpenNIGrabber* interface;

	friend class ofxPclStitcher;

	bool doColors;
};

#endif // OFXPCLSTITCHERDEVICE_H
