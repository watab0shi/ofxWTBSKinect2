#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
  kinect = new ofxKinect2::Device();
  kinect->setup();

  if( colorStream.setup( *kinect ) )     colorStream.open();
  if( depthStream.setup( *kinect ) )     depthStream.open();
  if( irStream.setup( *kinect ) )        irStream.open();
  if( bodyIndexStream.setup( *kinect ) ) bodyIndexStream.open();
  if( bodyStream.setup( *kinect ) )      bodyStream.open();
  
  ofSetFrameRate( 60 );
}

//--------------------------------------------------------------
void ofApp::update()
{
  kinect->update();
}

//--------------------------------------------------------------
void ofApp::draw()
{ 
  colorStream.draw();
  depthStream.draw( 0, 1080 - 424 );
  irStream.draw( 512, 1080 - 424 );
  bodyIndexStream.draw( 1024, 1080 - 424 );
  bodyStream.draw();

  if( bodyStream.getBodies().size() ) ofDrawBitmapStringHighlight( ofToString( ofGetFrameRate(), 2 ), 100, 100 );
}

//--------------------------------------------------------------
void ofApp::keyPressed( int key )
{

}

//--------------------------------------------------------------
void ofApp::keyReleased( int key )
{

}

//--------------------------------------------------------------
void ofApp::mouseMoved( int x, int y )
{

}

//--------------------------------------------------------------
void ofApp::mouseDragged( int x, int y, int button )
{

}

//--------------------------------------------------------------
void ofApp::mousePressed( int x, int y, int button )
{

}

//--------------------------------------------------------------
void ofApp::mouseReleased( int x, int y, int button )
{

}

//--------------------------------------------------------------
void ofApp::windowResized( int w, int h )
{

}

//--------------------------------------------------------------
void ofApp::gotMessage( ofMessage msg )
{

}

//--------------------------------------------------------------
void ofApp::dragEvent( ofDragInfo dragInfo )
{

}

//--------------------------------------------------------------
void ofApp::exit()
{
  colorStream.close();
  depthStream.close();
  irStream.close();
  bodyIndexStream.close();
  bodyStream.close();

  kinect->exit();
  delete kinect;
  kinect = nullptr;
}