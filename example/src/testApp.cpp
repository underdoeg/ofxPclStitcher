#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
	
	stitcher.setup(true, false);
	stitcher.doCalibrate = true;

	toggleCalibrate.setup(stitcher.doCalibrate);
	toggleCalibrate.setPosition(10, 10);
}

//--------------------------------------------------------------
void testApp::update(){
	stitcher.update();
}

//--------------------------------------------------------------
void testApp::draw(){
	ofBackground(0);
	stitcher.draw();

	ofSetColor(255);
	toggleCalibrate.draw();

	ofDrawBitmapStringHighlight("FPS "+ofToString(ofGetFrameRate()), 20, ofGetHeight()-20);
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){
	if(key == 'd')
		stitcher.toggleDebug();
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}
