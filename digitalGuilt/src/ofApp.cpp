#include "ofApp.h"
#include "robotCommandForward.h"
#include "robotCommandBackward.h"
#include "robotCommandTimedForward.h"
#include "robotCommandStop.h"
#include "robotCommandTurnOnSpotRight.h"
#include "robotCommandTurnOnSpotLeft.h"
#include "robotCommandFaceCompass.h"
#include "robotCommandHalfturn.h"


const string ofApp::OSC_HOSTNAME = "localhost";//"10.50.239.223";//
const int ofApp::OSC_PORT = 66666;


const int ofApp::ROBOT_FACING_TOLERANCE = 3;

const string ofApp::WIDGETNAME_SERIAL_DEVICE_DROPDOWN = "serialdevices_dropdown";
const string ofApp::WIDGETNAME_COMPASSVALUE_TEXTAREA = "compassvalue_textarea";
const string ofApp::WIDGETNAME_NBBLOBS_TEXTAREA = "nbblobs_textarea";
const string ofApp::WIDGETNAME_BLOB0X_TEXTAREA = "blob0x_textarea";
const string ofApp::WIDGETNAME_BLOB0Y_TEXTAREA = "blob0y_textarea";
const string ofApp::WIDGETNAME_ROBOTSTOP_BUTTON = "robotstop_button";
const string ofApp::WIDGETNAME_ROBOTFORWARD_BUTTON = "robotforward_button";
const string ofApp::WIDGETNAME_ROBOTBACKWARD_BUTTON = "robotbackward_button";
const string ofApp::WIDGETNAME_ROBOTTURNONSPOTLEFT_BUTTON = "robotturnonspotleft_button";
const string ofApp::WIDGETNAME_ROBOTTURNONSPOTRIGHT_BUTTON = "robotturnonspotright_button";
const string ofApp::WIDGETNAME_ROBOTFACEDIRECTION_BUTTON = "robotfacedirection_button";
const string ofApp::WIDGETNAME_ROBOTFACEDIRECTION_SLIDER = "robotfacedirection_slider";

const int ofApp::OPENCV_SURFACE_WIDTH = 640;//320;
const int ofApp::OPENCV_SURFACE_HEIGHT = 480;//240;
const int ofApp::OPENCV_CONTOUR_THRESHOLD = 3; 




//--------------------------------------------------------------
void ofApp::setup(){

	//////////////////////////////////////////////////////////////////////////
	//OSC
	// open an outgoing connection to HOST:PORT
	mOSCSender.setup(OSC_HOSTNAME, OSC_PORT);
	//////////////////////////////////////////////////////////////////////////
	//OpenCV
	mVidGrabber.setVerbose(true);
	//vidGrabber.videoSettings.
	mVidGrabber.initGrabber(OPENCV_SURFACE_WIDTH,OPENCV_SURFACE_HEIGHT);
	//vidGrabber.videoSettings();

	mColorImg.allocate(OPENCV_SURFACE_WIDTH,OPENCV_SURFACE_HEIGHT);
	mGrayImage.allocate(OPENCV_SURFACE_WIDTH,OPENCV_SURFACE_HEIGHT);
	mGrayBg.allocate(OPENCV_SURFACE_WIDTH,OPENCV_SURFACE_HEIGHT);
	mGrayDiff.allocate(OPENCV_SURFACE_WIDTH,OPENCV_SURFACE_HEIGHT);

	mLearnBakground = true;


	//////////////////////////////////////////////////////////////////////////
	//Serial
	mSynchedOnFirstLn = false;

	mSerial.listDevices();
	mSerialDeviceList = mSerial.getDeviceList();

	std::vector<string> deviceListString;// = new vector<string>();

	for (vector<ofSerialDeviceInfo>::iterator itt = mSerialDeviceList.begin(); itt  != mSerialDeviceList.end(); ++itt)
	{
		
		deviceListString.push_back(itt->getDevicePath());

	}
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	//UI
	//ofxUI IS DEPRECATED. USE ofxGUI INSTEAD
	/*mUI = new ofxUICanvas();        //Creates a canvas at (0,0) using the default width 

	mUI->addDropDownList(WIDGETNAME_SERIAL_DEVICE_DROPDOWN,deviceListString);

	mUI->addSlider("BACKGROUND",0.0,255.0,100.0); 

	mUI->addLabelButton(WIDGETNAME_ROBOTSTOP_BUTTON,false);
	mUI->addLabelButton(WIDGETNAME_ROBOTFORWARD_BUTTON,false);
	mUI->addLabelButton(WIDGETNAME_ROBOTBACKWARD_BUTTON,false);
	mUI->addLabelButton(WIDGETNAME_ROBOTTURNONSPOTLEFT_BUTTON,false);
	mUI->addLabelButton(WIDGETNAME_ROBOTTURNONSPOTRIGHT_BUTTON,false);
	mUI->addLabelButton(WIDGETNAME_ROBOTFACEDIRECTION_BUTTON,false);
	mUI->addBiLabelSlider(WIDGETNAME_ROBOTFACEDIRECTION_SLIDER, "0", "360", 0.f, 360.f, 180.f);



	mUI->addTextArea(WIDGETNAME_COMPASSVALUE_TEXTAREA,"test");

	mUI->addTextArea(WIDGETNAME_NBBLOBS_TEXTAREA, "No blobs");
	mUI->addTextArea(WIDGETNAME_BLOB0X_TEXTAREA, "-1");
	mUI->addTextArea(WIDGETNAME_BLOB0Y_TEXTAREA, "-1");

	mUI->addToggle("FULLSCREEN", false);


	mUI->autoSizeToFitWidgets(); 
	//ofAddListener(mUI->newGUIEvent, this, &ofApp::guiEvent); 
	mUI->loadSettings("settings.xml");*/
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	//Variables
	mLastValidRobotCompass = mRobotCompass = -1;

	mLongRobotCommand = NULL;
}

//--------------------------------------------------------------
void ofApp::update(){

	//((ofxUITextArea*) mUI->getWidget("OUT_TEXT"))->setTextString("Updated");
	//////////////////////////////////////////////////////////////////////////
	//OpenCV
	bool bNewFrame = false;

	mVidGrabber.update();
	bNewFrame = mVidGrabber.isFrameNew();

	if (bNewFrame)
	{
		mColorImg.setFromPixels(mVidGrabber.getPixels(), OPENCV_SURFACE_WIDTH, OPENCV_SURFACE_HEIGHT);

		mGrayImage = mColorImg;
		if (mLearnBakground == true){
			mGrayBg = mGrayImage;		// the = sign copys the pixels from grayImage into grayBg (operator overloading)
			mLearnBakground = false;
		}

		// take the abs value of the difference between background and incoming and then threshold:
		mGrayDiff.absDiff(mGrayBg, mGrayImage);
		mGrayDiff.threshold(OPENCV_CONTOUR_THRESHOLD);

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		mContourFinder.findContours(mGrayDiff, 20, ((OPENCV_SURFACE_WIDTH+20) /*340*/*OPENCV_SURFACE_HEIGHT)/3, 10, false);//true);	// find holes

		//((ofxUITextArea*) mUI->getWidget(WIDGETNAME_NBBLOBS_TEXTAREA))->setTextString(to_string(mContourFinder.nBlobs));

		if (mContourFinder.nBlobs != 0)
		{
			int posX = mContourFinder.blobs[0].centroid.x;
			int posY = mContourFinder.blobs[0].centroid.y;
			//((ofxUITextArea*) mUI->getWidget(WIDGETNAME_BLOB0X_TEXTAREA))->setTextString("x : " + to_string(posX));
			//((ofxUITextArea*) mUI->getWidget(WIDGETNAME_BLOB0Y_TEXTAREA))->setTextString("y : " + to_string(posY));

			ofxOscMessage m;
			m.setAddress("/robot/position_xy");
			m.addIntArg(posX);
			m.addIntArg(posY);

			mOSCSender.sendMessage(m);
			
		}
	}
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	//Robot readings and control
	mRobotCompass = getSyncedRobotCompass();

	if (mRobotCompass != -1)
	{
		//((ofxUITextArea*) mUI->getWidget(WIDGETNAME_COMPASSVALUE_TEXTAREA))->setTextString(to_string(mRobotCompass));

		ofxOscMessage m;
		m.setAddress("/robot/compass");
		m.addIntArg(mRobotCompass);
		
		mOSCSender.sendMessage(m);

	}

	if (mLongRobotCommand != NULL)
	{
		if (!mLongRobotCommand->onUpdate())
		{
			mLongRobotCommand->onEnd();
			delete mLongRobotCommand;
			mLongRobotCommand = NULL;
		}
	}
	//////////////////////////////////////////////////////////////////////////


}

//Retrieves the robot compass value as an int
//or -1 if it's not available. also takes care or syncing over serial port
int ofApp::getSyncedRobotCompass(){

	int toReturn = -1;

	if (mSerial.isInitialized() && mSerial.available() >3 && !mSynchedOnFirstLn)
	{
		syncSerial();
	}

	if (mSerial.isInitialized() && mSynchedOnFirstLn && mSerial.available())
	{
		//Try to read at most 4 bytes
		const int bytesAtMost = 5;

		string compassValueString = "";
		
		
		bool lnFound = false;

		int bytesActualyRead = 0;

		unsigned char lastRead[1];
		//int bytesRemaining = bytesRequired;
		// loop until we've read everything
		while ( !lnFound && bytesActualyRead < bytesAtMost )
		{
		  
			//Read one byte
			int result = mSerial.readBytes( &lastRead[0],
				1 );
 
			// check for error code
			if ( result == OF_SERIAL_ERROR )
			{
				// something bad happened
				ofLog( OF_LOG_ERROR, "unrecoverable error reading from serial" );
				// bail out
				break;
			}
			else if ( result == OF_SERIAL_NO_DATA )
			{
				// nothing was read, try again
			}
			else
			{
				// we read some data!
				if (result == 0)
				{
					continue;
				}

				string lastReadString(lastRead, lastRead+1);

				if (lastReadString == "\n")
				{
					lnFound = true;
				}
				else if (lastReadString == "\r")
				{}
				else
				{
					compassValueString += lastReadString;
				}

				bytesActualyRead += result;
			}
		  
		}

		//Data available in bytes
		//construct string
		//string compassValueString(bytes, bytes + 3); //drop \n for textArea display

		//((ofxUITextArea*) mUI->getWidget(WIDGETNAME_OUT_TEXTAREA))->setTextString(compassValueString);

		if (lnFound)
		{
			toReturn = stoi(compassValueString);
			mLastValidRobotCompass = toReturn;
		}
	}

	return toReturn;

}

void ofApp::syncSerial(){

	unsigned char byte[1];

	int result = mSerial.readBytes(byte,1);

	//bool toReturn = false;

	// check for error code
    if ( result == OF_SERIAL_ERROR )
    {
      // something bad happened
      ofLog( OF_LOG_ERROR, "unrecoverable error reading from serial" );
      // bail out
    }
    else if ( result == OF_SERIAL_NO_DATA )
    {
      // nothing was read, try again
    }
    else
    {
		std::string my_std_string(byte, byte + 1);
		//string s = reinterpret_cast<unsigned char*>(byte[0]);
      // we read some data!
		if (my_std_string == "\n")
		{
			//We're synced !
			//mSerial.writeBytes(reinterpret_cast<unsigned char*>("2\n"),2);
			
			
			/*mSerial.writeByte('2');
			mSerial.writeByte('\r');

			mSerial.writeByte('\n');*/
			//mSerial.writeBytes(ROBOT_COMMAND_TURNONSPOT,3);
			//orderRobotTurnOnSpot();
			
			mSynchedOnFirstLn = true;

		}

		

		

		
      
    }
}

//--------------------------------------------------------------
void ofApp::draw(){

	//////////////////////////////////////////////////////////////////////////
	//OpenCV
	// draw the incoming, the grayscale, the bg and the thresholded difference
	ofSetHexColor(0xffffff);
	//mColorImg.draw(20,20);
	//mGrayImage.draw(360,20);
	//mGrayBg.draw(20,280);
	
	
	//mGrayDiff.draw(360,280);
	//Sould be nice to display this content through some Widget of ofoxUI
	mGrayDiff.draw(20,20);

	//ofFill();
	//ofSetHexColor(0x333333);
	//ofRect(OPENCV_SURFACE_WIDTH+20/*360*/,OPENCV_SURFACE_WIDTH+300/*540*/,OPENCV_SURFACE_WIDTH,OPENCV_SURFACE_WIDTH);
	//ofSetHexColor(0xffffff);

	// we could draw the whole contour finder
	//mContourFinder.draw(360,280);  //Parameters are X,Y absolute coordinate on screen
	mContourFinder.draw(20,20);  //Parameters are X,Y absolute coordinate on screen


	// or, instead we can draw each blob individually from the blobs vector,
	// this is how to get access to them:
	/*for (int i = 0; i < mContourFinder.nBlobs; i++){
		mContourFinder.blobs[i].draw(360,280);

		// draw over the centroid if the blob is a hole
		ofSetColor(255);
		if(mContourFinder.blobs[i].hole){
			ofDrawBitmapString("hole",
				mContourFinder.blobs[i].boundingRect.getCenter().x + 360,
				mContourFinder.blobs[i].boundingRect.getCenter().y + 540);
		}
	}*/

	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	//UI
	//d - circle
	ofSetHexColor(0xCC0000);
	ofDrawBitmapString("d) circle(); \n", 380, 140);
	mVectorGraphics.setColor(0x828282);

	float circleCenterX = 425.f;
	float circleCenterY = 80.0f;
	float circleRadius = 30.f;


	if (mLastValidRobotCompass != -1)
	{
		mVectorGraphics.circle(circleCenterX, circleCenterY, circleRadius);

		//i - just a line
		//ofSetHexColor(0xCC0000);
		//ofDrawBitmapString("i) line(); \n", 530, 300);

		float onCircleX = circleCenterX + circleRadius * cosf(ofDegToRad(mLastValidRobotCompass));
		float onCircleY = circleCenterY + circleRadius * sinf(ofDegToRad(mLastValidRobotCompass));



		mVectorGraphics.setColor(0xCC0000);
		mVectorGraphics.line(circleCenterX, circleCenterY, onCircleX, onCircleY);
	}
	

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key){
	case ' ':
		mLearnBakground = true;
		break;
	case 'a':
		robotCommandTurnOnSpotLeft(mSerial).onStart();
		break;
	case 'd':
		robotCommandTurnOnSpotRight(mSerial).onStart();
		break;
	case 'w':
		robotCommandForward(mSerial).onStart();
		break;
	case 's':
		robotCommandStop(mSerial).onStart();
		break;
	}

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
void ofApp::exit()
{
	//mUI->saveSettings("settings.xml");     
    //delete mUI;

}

/*void ofApp::guiEvent(ofxUIEventArgs &e)
{
	if(e.getName() == "BACKGROUND")
    {
        ofxUISlider *slider = e.getSlider();    
        ofBackground(slider->getScaledValue());
    }
    else if(e.getName() == "FULLSCREEN")
    {
        ofxUIToggle *toggle = e.getToggle(); 
        ofSetFullscreen(toggle->getValue()); 
    }  
	else if (e.getName() == WIDGETNAME_SERIAL_DEVICE_DROPDOWN)
	{

		ofxUIDropDownList* listWidget = (ofxUIDropDownList*)e.widget;
		vector<string> selectedNames = listWidget->getSelectedNames();

		if (selectedNames.size() == 1)
		{
			mSerial.setup(selectedNames.front(), 57600);
		}
	}
	else if(e.getName() == WIDGETNAME_ROBOTSTOP_BUTTON){
		robotCommandStop command = robotCommandStop(mSerial);
		command.onStart();
	}
	else if(e.getName() == WIDGETNAME_ROBOTFORWARD_BUTTON){
		
		//mLongRobotCommand = new robotCommandTimedForward(mSerial,1000.f);
		//mLongRobotCommand->onStart();
		robotCommandForward(mSerial).onStart();
		
	} else if(e.getName() == WIDGETNAME_ROBOTBACKWARD_BUTTON){

		//mLongRobotCommand = new robotCommandTimedForward(mSerial,1000.f);
		//mLongRobotCommand->onStart();
		robotCommandBackward(mSerial).onStart();

	}
	else if(e.getName() == WIDGETNAME_ROBOTTURNONSPOTLEFT_BUTTON){
		robotCommandTurnOnSpotLeft command = robotCommandTurnOnSpotLeft(mSerial);
		command.onStart();
	}
	else if(e.getName() == WIDGETNAME_ROBOTTURNONSPOTRIGHT_BUTTON ){

		if (mLongRobotCommand == NULL)
		{
			mLongRobotCommand = new robotCommandHalfturn(ROBOT_FACING_TOLERANCE, mLastValidRobotCompass, mSerial);
			mLongRobotCommand->onStart();
		}
		
		//robotCommandTurnOnSpotRight command = robotCommandTurnOnSpotRight(mSerial);
		//command.onStart();
	}
	else if (e.getName() == WIDGETNAME_ROBOTFACEDIRECTION_BUTTON)
	{
		ofxUISlider *slider = (ofxUISlider*)mUI->getWidget(WIDGETNAME_ROBOTFACEDIRECTION_SLIDER); 
		
		int facingRequiredByUI = (int)(slider->getValue());
		mLongRobotCommand = new robotCommandFaceCompass(facingRequiredByUI,ROBOT_FACING_TOLERANCE, mLastValidRobotCompass,mSerial);
		mLongRobotCommand->onStart();
	}

}*/
