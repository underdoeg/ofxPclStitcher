#ifndef OFXPCLSTITCHERDEVICE_H
#define OFXPCLSTITCHERDEVICE_H

#include <XnCppWrapper.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>

#include "ofMain.h"

class ofxPclStitcher;

typedef pcl::PointXYZ ofxPclPoint;
typedef pcl::PointXYZRGBA ofxPclPointColor;

typedef pcl::PointCloud<ofxPclPoint> ofxPclCloud;
typedef pcl::PointCloud<ofxPclPointColor> ofxPclCloudColor;

typedef ofxPclCloud::Ptr ofxPclCloudPtr;
typedef ofxPclCloudColor::Ptr ofxPclCloudPtrColor;

typedef ofxPclCloud::ConstPtr ofxPclCloudConstPtr;
typedef ofxPclCloudColor::ConstPtr ofxPclCloudConstPtrColor;


class ofxPclStitcherDevice {
public:
	~ofxPclStitcherDevice();

private:
	void cloudCallbackColor(const ofxPclCloudConstPtrColor cloudIn);
	void cloudCallback(const ofxPclCloudConstPtr cloudIn);
	 /*
	 {
		//thread safety first
		mutex.lock();
		pcl::copyPointCloud(*cloud_, *cloudThread);
		mutex.unlock();
	}
*/

	ofxPclStitcherDevice(string address, bool doColors);

	pcl::OpenNIGrabber* interface;

	friend class ofxPclStitcher;

	bool doColors;

	ofMutex mutex;

	ofxPclCloudPtr cloudThread;
	ofxPclCloudPtrColor cloudThreadColor;
};

#endif // OFXPCLSTITCHERDEVICE_H
