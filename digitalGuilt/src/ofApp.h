#pragma once

#include "ofxVectorGraphics.h"
#include "ofMain.h"
//#include "ofxUI.h"
#include "ofxOpenCv.h"
#include "ofxOsc.h"

class robotCommandBase;

class ofApp : public ofBaseApp{

	//////////////////////////////////////////////////////////////////////////
	//CONSTANTS
	//////////////////////////////////////////////////////////////////////////
	//ROBOT
	static const int ROBOT_FACING_TOLERANCE;
	//UI
	static const string WIDGETNAME_SERIAL_DEVICE_DROPDOWN;
	static const string WIDGETNAME_COMPASSVALUE_TEXTAREA;
	static const string WIDGETNAME_NBBLOBS_TEXTAREA;
	static const string WIDGETNAME_BLOB0X_TEXTAREA;
	static const string WIDGETNAME_BLOB0Y_TEXTAREA;
	static const string WIDGETNAME_ROBOTSTOP_BUTTON;
	static const string WIDGETNAME_ROBOTFORWARD_BUTTON;
	static const string WIDGETNAME_ROBOTBACKWARD_BUTTON;
	static const string WIDGETNAME_ROBOTTURNONSPOTLEFT_BUTTON;
	static const string WIDGETNAME_ROBOTTURNONSPOTRIGHT_BUTTON;
	static const string WIDGETNAME_ROBOTFACEDIRECTION_BUTTON;
	static const string WIDGETNAME_ROBOTFACEDIRECTION_SLIDER;
	//OpenCV
	static const int OPENCV_CONTOUR_THRESHOLD;
	static const int OPENCV_SURFACE_HEIGHT;
	static const int OPENCV_SURFACE_WIDTH;
	//OSC
	static const string OSC_HOSTNAME;
	static const int OSC_PORT;
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	//UI
	//ofxUICanvas* mUI;
	ofxVectorGraphics mVectorGraphics;
	public:
		void exit(); 
		//void guiEvent(ofxUIEventArgs &e);
		
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

private:
	//////////////////////////////////////////////////////////////////////////
	//Serial and robot
		vector <ofSerialDeviceInfo> mSerialDeviceList;

		ofSerial	mSerial;

		int mRobotCompass;
		int mLastValidRobotCompass;	//stores compass value when reading gives -1
		bool mSynchedOnFirstLn;

		void syncSerial();
		int getSyncedRobotCompass();

		//To update a long running command
		robotCommandBase* mLongRobotCommand;
		//////////////////////////////////////////////////////////////////////////
		//openCV
		ofVideoGrabber 		mVidGrabber;

		ofxCvColorImage			mColorImg;

		ofxCvGrayscaleImage 	mGrayImage;
		ofxCvGrayscaleImage 	mGrayBg;
		ofxCvGrayscaleImage 	mGrayDiff;

		ofxCvContourFinder 	mContourFinder;

		//int 				threshold;
		bool				mLearnBakground;
		//////////////////////////////////////////////////////////////////////////
		//OSC
		ofxOscSender mOSCSender;
};
