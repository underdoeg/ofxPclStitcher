#ifndef OFXPCLSTITCHER_H
#define OFXPCLSTITCHER_H

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

	ofParameter<bool> downsample;
	ofParameter<float> downsampleSize;
	ofParameter<bool> debug;
	ofParameter<float> scale;

private:
	typedef std::vector< ofPtr<ofxPclStitcherDevice> > DeviceList;
	DeviceList devices;
	unsigned int curDeviceNumber;
	ofParameterGroup parameters;
	ofParameter<bool> doColors;
	ofEasyCam cam;

	string settingsFilename;

	ofxPanel gui;

};

#endif // OFXPCLSTITCHER_H
