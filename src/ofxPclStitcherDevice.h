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

typedef typename ofxPclCloud::Ptr ofxPclCloudPtr;
typedef typename ofxPclCloudColor::Ptr ofxPclCloudPtrColor;

typedef typename ofxPclCloud::ConstPtr ofxPclCloudConstPtr;
typedef typename ofxPclCloudColor::ConstPtr ofxPclCloudConstPtrColor;


class ofxPclStitcherDevice {
public:
	~ofxPclStitcherDevice();

private:
	void cloudCallbackColor(const ofxPclCloudPtrColor cloudIn);
	void cloudCallback(const ofxPclCloudPtr cloudIn);
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
