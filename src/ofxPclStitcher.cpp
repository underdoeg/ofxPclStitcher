#include "ofxPclStitcher.h"

ofxPclStitcher::ofxPclStitcher():curDeviceNumber(1) {
}

ofxPclStitcher::~ofxPclStitcher() {
}

void ofxPclStitcher::setup(bool autoCreateDevices, bool dc) {

	debug = false;

	doColors = dc;
	downsample.set("DOWNSAMPLE", true);
	downsampleSize.set("DOWNSAMPLE SIZE", .3, 0.001, .1);
	scale.set("SCALE", 100, 0.1, 1000);

	settingsFilename = "pclStitcherSettings.xml";

	gui.setup("PCL STITCHER SETTINGS", settingsFilename);
	gui.add(downsample);
	gui.add(downsampleSize);
	gui.loadFromFile(settingsFilename);

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

ofxPclStitcherDevice* ofxPclStitcher::createDevice()
{
	curDeviceNumber++;
	return createDevice(curDeviceNumber-1);
}

ofxPclStitcherDevice* ofxPclStitcher::createDevice(int number)
{
	return createDevice("#"+ofToString(number));
}

ofxPclStitcherDevice* ofxPclStitcher::createDevice(string address)
{
	ofxPclStitcherDevice* device = new ofxPclStitcherDevice(address, doColors);
	device->downsample.makeReferenceTo(downsample);
	device->downsampleSize.makeReferenceTo(downsampleSize);
	device->scale.makeReferenceTo(scale);
	device->debug.makeReferenceTo(debug);
	gui.add(device->parameters);
	gui.loadFromFile(settingsFilename);
	devices.push_back(ofPtr<ofxPclStitcherDevice>(device));
	return device;
}

void ofxPclStitcher::update()
{
	//copy clouds over from thread
	for(DeviceList::iterator it = devices.begin();it != devices.end();it++){
		(*it)->copyCloudFromThread();
	}

	for(DeviceList::iterator it = devices.begin();it != devices.end();it++){
		(*it)->processCloud();
	}
}

void ofxPclStitcher::draw()
{
	ofPushStyle();
	cam.begin();
	glEnable(GL_DEPTH_TEST);
	ofEnableAlphaBlending();
	for(DeviceList::iterator it = devices.begin();it != devices.end();it++){
		(*it)->draw();
	}
	cam.end();
	glDisable(GL_DEPTH_TEST);
	gui.draw();
	for(DeviceList::iterator it = devices.begin();it != devices.end();it++){
		(*it)->drawOverlay();
	}
	ofPopStyle();

}

void ofxPclStitcher::toggleDebug()
{
	debug = !debug;
}
