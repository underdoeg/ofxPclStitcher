#include "ofxPclStitcher.h"

#include "ofxPclUtils.h"

ofxPclStitcher::ofxPclStitcher():curDeviceNumber(1) {
}

ofxPclStitcher::~ofxPclStitcher() {
}

void ofxPclStitcher::setup(bool autoCreateDevices, bool dc) {

	doCalibrate.set("CALIBRATE", false);
	doNoiseReduction.set("NOISE REDUCTION", false);

	doColors = dc;
	doDownsample.set("DOWNSAMPLE", true);
	downsampleSize.set("DOWNSAMPLE SIZE", .3, 0.001, .1);
	scale.set("SCALE", 100, 0.1, 1000);

	doConcaveHull.set("CONCAVE HULL", false);
	concaveHullSize.set("CONCAVE HULL SIZE", .1, .0001, .5);

	doTriangulation.set("TRIANGULATION", false);
	triangulationRadius.set("TRIANGULATION RADIUS", .03, .0001, 3);

	drawGrid.set("DRAW GRID", true);

	settingsFilename = "pclStitcherSettings.xml";

	guiWidth = 350;
	gui.setup("PCL STITCHER SETTINGS", settingsFilename);
	gui.setSize(guiWidth, gui.getHeight());
	gui.add(drawGrid);
	gui.add(doDownsample);
	gui.add(downsampleSize);
	gui.add(doNoiseReduction);
	gui.add(doTriangulation);
	gui.add(triangulationRadius);
	gui.add(doConcaveHull);
	gui.loadFromFile(settingsFilename);
	gui.setPosition(10, 40);
	gui.setWidthElements(guiWidth);

	cloud = ofxPclCloudPtr(new ofxPclCloud());
	cloudColor = ofxPclCloudPtrColor(new ofxPclCloudColor());
	normals =  pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

	searchTree = pcl::search::KdTree<ofxPclPoint>::Ptr(new pcl::search::KdTree<ofxPclPoint>);
	searchTreeColor = pcl::search::KdTree<ofxPclPointColor>::Ptr(new pcl::search::KdTree<ofxPclPointColor>);

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

	ofAddListener(ofEvents().keyPressed, this, &ofxPclStitcher::keyPressed);
	ofAddListener(ofEvents().keyReleased, this, &ofxPclStitcher::keyReleased);

	firstDraw = 0;

	//camTop.enableOrtho();
	camTop.setPosition(ofVec3f(1, -200, 1));
	camTop.lookAt(ofVec3f(1, 1, 1));

	//camLeft.enableOrtho();
	camLeft.setPosition(ofVec3f(200, 0, 0));
	camLeft.lookAt(ofVec3f(0, 0, 0));

	//camFront.enableOrtho();
	camFront.setPosition(ofVec3f(0, 0, 200));
	camFront.lookAt(ofVec3f(0, 0, 0));

	cam = &easyCam;
}

ofxPclStitcherDevice* ofxPclStitcher::createDevice() {
	curDeviceNumber++;
	return createDevice(curDeviceNumber-1);
}

ofxPclStitcherDevice* ofxPclStitcher::createDevice(int number) {
	return createDevice("#"+ofToString(number));
}

ofxPclStitcherDevice* ofxPclStitcher::createDevice(string address) {
	ofxPclStitcherDevice* device = new ofxPclStitcherDevice(address, doColors);
	device->downsample.makeReferenceTo(doDownsample);
	device->downsampleSize.makeReferenceTo(downsampleSize);
	device->scale.makeReferenceTo(scale);
	device->debug.makeReferenceTo(doCalibrate);
	gui.add(device->parameters);
	ofxBaseGui* kGui = gui.getControl(gui.getNumControls()-1);
	kGui->setHeaderBackgroundColor(device->color);
	gui.loadFromFile(settingsFilename);
	devices.push_back(ofPtr<ofxPclStitcherDevice>(device));

	gui.setWidthElements(guiWidth);

	return device;
}

void ofxPclStitcher::update() {
	bool newData = true;
	for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
		if(!(*it)->hasNewData())
			newData = false;
	}

	if(!newData) {
		return;
	}

	for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
		(*it)->copyCloudFromThread();
	}

	//copy clouds over from thread
	for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
		(*it)->copyCloudFromThread();
	}

	for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
		(*it)->processCloud();
	}

	if(!doCalibrate) { //don't create the final mesh in debug mode to save performance

		cloud->clear();
		cloudColor->clear();

		for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
			if(doColors)
				*cloudColor += *(*it)->cloudColor;
			else
				if((*it)->doDraw)
					*cloud += *(*it)->cloud;
		}

		ofEventArgs a;
		ofNotifyEvent(onPreprocess, a);

		//downsample if needed
		if(doDownsample) {
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



		//construct the concave hull
		if(doConcaveHull) {
			if(doColors) {
				concaveHullColor.setInputCloud(cloudColor);
				concaveHullColor.setAlpha(concaveHullSize);
				concaveHullColor.reconstruct(*cloudColor);
			} else {
				concaveHull.setInputCloud(cloud);
				concaveHull.setAlpha(concaveHullSize);
				concaveHull.reconstruct(*cloud);
			}
		}

		if(doNoiseReduction) {
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

			pcl::PointCloud<pcl::PointNormal> mls_points;

			pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

			//mls.setComputeNormals(true);
			mls.setInputCloud (cloud);
			mls.setPolynomialFit (true);
			mls.setSearchMethod (tree);
			mls.setSearchRadius (0.03);

			mls.process(mls_points);

			pcl::copyPointCloud(mls_points, *cloud);
		}

		if(doTriangulation) {
			//first calculate normals
			//TODO: maybe some of those will also work in setup()
			normals->clear();
			searchTree->setInputCloud(cloud);
			normalEstimation.setInputCloud(cloud);
			normalEstimation.setSearchMethod(searchTree);
			normalEstimation.setKSearch(20);
			normalEstimation.compute(*normals);

			pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);

			// Create search tree*
			pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
			tree->setInputCloud (cloudWithNormals);

			gp3.setSearchRadius(triangulationRadius);

			// Set typical values for the parameters
			gp3.setMu (2.5);
			gp3.setMaximumNearestNeighbors (100);
			gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
			gp3.setMinimumAngle(M_PI/18); // 10 degrees
			gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
			gp3.setNormalConsistency(false);

			//get the tris
			gp3.setInputCloud(cloudWithNormals);
			gp3.setSearchMethod (tree);
			gp3.reconstruct(polygonMesh);
		}


		if(doColors) {
			toOf(cloudColor, mesh, scale, scale, scale);
			if(doTriangulation)
				addIndices(mesh, polygonMesh);
		} else {
			toOf(cloud, mesh, scale, scale, scale);
			if(doTriangulation)
				addIndices(mesh, polygonMesh);
		}


	}

}

void ofxPclStitcher::draw() {
	if(!doCalibrate)
		return;

	ofPushStyle();

	glEnable(GL_DEPTH_TEST);
	ofEnableAlphaBlending();

	//top Left
	//ofPushView();
	//easyCam.begin();

	cam->begin();

	//ofPushMatrix();
	if(cam != &easyCam){
		//ofTranslate(ofGetWidth()*.5, ofGetHeight()*.5);
	}

	draw3d();
	//easyCam.end();
	//ofPopMatrix();

	cam->end();

	//ofPopView();

	glDisable(GL_DEPTH_TEST);

	gui.draw();

	for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
		(*it)->drawOverlay();
	}

	ofPopStyle();

	if(firstDraw==4) {
		firstDraw = 100;
		easyCam.disableMouseInput();
	}
	firstDraw++;
}

void ofxPclStitcher::draw3d() {
	//ofDrawGrid(100, 13, false, true, false, false);
	ofPushMatrix();
	ofRotateZ(90);
	if(drawGrid)
		ofDrawGridPlane(100);
	ofPopMatrix();
	ofDrawAxis(100);
	for(DeviceList::iterator it = devices.begin(); it != devices.end(); it++) {
		(*it)->draw();
	}
	ofEventArgs e;
	ofNotifyEvent(onCalibrateDraw, e);
}

void ofxPclStitcher::toggleDebug() {
	doCalibrate = !doCalibrate;
}

void ofxPclStitcher::keyPressed(ofKeyEventArgs& e) {
	if(doCalibrate && e.key == 'c') {
		easyCam.enableMouseInput();
	}
}

void ofxPclStitcher::keyReleased(ofKeyEventArgs& e) {
	if(e.key == 'c') {
		easyCam.disableMouseInput();
	}else if(e.key == 49){
		cam = &camTop;
	}else if(e.key == 50){
		cam = &camFront;
	}else if(e.key == 51){
		cam = &camLeft;
	}else if(e.key == 52){
		cam = &easyCam;
	}else if(e.key == 'r'){

	}
}
