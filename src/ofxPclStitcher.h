#ifndef OFXPCLSTITCHER_H
#define OFXPCLSTITCHER_H

#include "ofxPclStitcherDevice.h"

class ofxPclStitcher
{
public:
	ofxPclStitcher();
	~ofxPclStitcher();

	void setup(bool autoCreateDevices = true, bool doColors = false);
	ofxPclStitcherDevice* createDevice(string address);
	ofxPclStitcherDevice* createDevice(int number);
	ofxPclStitcherDevice* createDevice();

private:
	std::vector< ofPtr<ofxPclStitcherDevice> > devices;
	unsigned int curDeviceNumber;
	bool doColors;
};

#endif // OFXPCLSTITCHER_H
