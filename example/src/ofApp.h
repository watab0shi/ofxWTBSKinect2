#pragma once

#include "ofMain.h"
#include "ofxKinect2.h"

class ofApp : public ofBaseApp
{
public:
  void setup();
  void update();
  void draw();

  void keyPressed( int key );
  void keyReleased( int key );
  void mouseMoved( int x, int y );
  void mouseDragged( int x, int y, int button );
  void mousePressed( int x, int y, int button );
  void mouseReleased( int x, int y, int button );
  void windowResized( int w, int h );
  void dragEvent( ofDragInfo dragInfo );
  void gotMessage( ofMessage msg );

  void exit();

  ofxKinect2::Device*         kinect;
  ofxKinect2::ColorStream     colorStream;
  ofxKinect2::DepthStream     depthStream;
  ofxKinect2::IrStream        irStream;
  ofxKinect2::BodyIndexStream bodyIndexStream;
  ofxKinect2::BodyStream      bodyStream;
};
